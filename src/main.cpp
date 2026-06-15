//
// Created by awalol on 2026/3/4.
//

#include <cstring>
#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "wake.h"
#ifdef ENABLE_WAKE_HID
#include "ps_shortcut.h"
#endif
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "state_mgr.h"
#if ENABLE_SERIAL
#include "pico/stdio_usb.h"
#endif
#include "config.h"
#include "cmd.h"
#include "ds5_input_tuning.h"
#include "ns2pro_ble.h"
#include "ns2pro_config.h"
#include "pico_led.h"
#include "ns2pro_rumble.h"
#include "ns2pro_serial_bridge.h"
#include "ns2pro_translator.h"
#include "usb.h"
#if ENABLE_BATT_LED
#include "battery_led.h"
#endif

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

#ifndef ENABLE_NS2PRO_SERIAL_BRIDGE
#define ENABLE_NS2PRO_SERIAL_BRIDGE 0
#endif

int reportSeqCounter = 0;
uint8_t packetCounter = 0;
bool spk_active = false;
bool ns2pro_serial_active = false;
uint32_t ns2pro_report_tick = 0;
uint32_t ns2pro_last_seen_ms = 0;
bool ns2pro_wired_bond_saved = false;
Ns2ProInputState ns2pro_ble_latest_state{};
bool ns2pro_ble_state_valid = false;
bool ns2pro_ble_state_pending = false;
uint64_t ns2pro_ble_next_dispatch_us = 0;
uint8_t ds5_split_output_report[63]{};
bool ds5_split_output_pending = false;
uint8_t ds5_feature_output_report[63]{};
bool ds5_feature_output_pending = false;

uint8_t interrupt_in_data[63] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

namespace {

constexpr size_t kDs5OutputReportLen = 63;
constexpr size_t kDs5SplitFirstChunkLen = 31;
constexpr size_t kDs5SplitSecondChunkLen = 32;
constexpr size_t kDs5FeatureFirstChunkLen = 62;
constexpr uint32_t kNs2ProInputActivityTimeoutMs = 1200;
constexpr uint32_t kBridgeStatusRefreshIntervalMs = 20;
constexpr uint64_t kNs2ProBleDispatchIntervalUs = 7400;

bool ns2pro_serial_transport_active(uint32_t now_ms) {
    (void) now_ms;
    // Once the PC-side serial bridge has been established and we have observed
    // NS2Pro traffic on it, keep the serial output path available for rumble.
    // Game rumble can arrive while the controller is stationary, so gating the
    // output route on "recent input activity within 1200ms" drops short haptics.
    return ns2pro_serial_bridge_enabled() &&
           ns2pro_serial_bridge_has_activity();
}

bool send_ns2pro_output_report(const uint8_t *report, size_t len) {
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (ns2pro_ble_connected()) {
        const bool queued = ns2pro_ble_send_output_report(report, len);
        ns2pro_rumble_note_output_attempt(queued, ns2pro_ble_last_output_status());
        if (queued) {
            ns2pro_rumble_note_output_sent(false);
        }
        return queued;
    }

    if (ns2pro_serial_transport_active(now_ms)) {
        const bool queued = ns2pro_serial_bridge_send_output_report(report, len);
        ns2pro_rumble_note_output_attempt(queued, queued ? 3 : ns2pro_serial_bridge_last_output_status());
        if (queued) {
            ns2pro_rumble_note_output_sent(true);
        }
        return queued;
    }

    ns2pro_rumble_note_output_attempt(false, 0x20);
    return false;
}

void handle_complete_ds5_output_report(const uint8_t *report, uint16_t len) {
    if (!report || len == 0 || report[0] != 0x02) {
        return;
    }

    ns2pro_rumble_on_ds5_output_report(report, len);
    uint8_t rumble_report[64];
    if (ns2pro_rumble_build_output_report(rumble_report)) {
        send_ns2pro_output_report(rumble_report, sizeof(rumble_report));
    }
    if (len > 1) {
        state_update(report + 1, len - 1);
    }
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    if (ns2pro_serial_active) {
        // When NS2Pro serial input is actively owning the bridge, translate
        // DS5 output feedback into NS2Pro-compatible packets and keep it on
        // the serial bridge path only. When NS2Pro is not the active source,
        // preserve the original DS5 local feedback path unchanged.
        return;
    }
#endif
    if (spk_active) {
        return;
    }
    uint8_t outputData[78]{};
    outputData[0] = 0x31;
    outputData[1] = reportSeqCounter << 4;
    if (++reportSeqCounter == 256) {
        reportSeqCounter = 0;
    }
    outputData[2] = 0x10;
    state_set(outputData + 3, sizeof(SetStateData));
    bt_write(outputData, sizeof(outputData));
}

bool ns2pro_transport_present(uint32_t now_ms) {
    const uint32_t last_serial_input_ms = ns2pro_serial_bridge_last_input_ms();
    const bool serial_present =
        last_serial_input_ms != 0 &&
        (now_ms - last_serial_input_ms) <= kNs2ProInputActivityTimeoutMs;
    return ns2pro_ble_connected() || serial_present;
}

bool should_service_ns2pro_runtime(uint32_t now_ms) {
    if (ns2pro_transport_present(now_ms) || ns2pro_serial_active) {
        return true;
    }

    return !bt_classic_connected();
}

void sync_pairing_services(uint32_t now_ms) {
    const bool ns2pro_present = ns2pro_transport_present(now_ms) || ns2pro_serial_active;
    const bool ds5_present = bt_classic_connected();
    const bool any_controller_present = ds5_present || ns2pro_present;

    if (any_controller_present) {
        bt_set_classic_pairing_service_enabled(false);
        ns2pro_ble_set_pairing_service_enabled(false);
        ns2pro_ble_set_idle_pairing_enabled(false);
        return;
    }

    bt_set_classic_pairing_service_enabled(true);
    ns2pro_ble_set_pairing_service_enabled(true);
    ns2pro_ble_set_idle_pairing_enabled(true);
}

bool should_stream_continuous_usb_reports(uint32_t now_ms) {
    (void) now_ms;
    return true;
}

void sync_usb_bridge_connection(uint32_t now_ms) {
    const bool ns2pro_present = ns2pro_transport_present(now_ms);
    const bool any_controller_present =
        bt_classic_connected() || ns2pro_present || usb_bridge_should_hold_dualsense();
    usb_bridge_request_persona(
        any_controller_present ? USB_BRIDGE_PERSONA_DUALSENSE : USB_BRIDGE_PERSONA_MANAGER
    );
}

bool try_handle_ds5_output_transport(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize) {
    if (!buffer || bufsize == 0) {
        return false;
    }

    if (report_id == 0 && buffer[0] == 0x02) {
        handle_complete_ds5_output_report(buffer, bufsize);
        return true;
    }

    if (report_id == 0x02) {
        if (buffer[0] == 0x02) {
            handle_complete_ds5_output_report(buffer, bufsize);
            return true;
        }
        if (buffer[0] == 0xFD) {
            std::memset(ds5_split_output_report, 0, sizeof(ds5_split_output_report));
            const size_t copy_len = std::min(static_cast<size_t>(bufsize > 1 ? bufsize - 1 : 0), kDs5SplitFirstChunkLen);
            std::memcpy(ds5_split_output_report, buffer + 1, copy_len);
            ds5_split_output_pending = true;
            return true;
        }
        if (buffer[0] == 0xFE) {
            if (!ds5_split_output_pending) {
                return true;
            }
            const size_t copy_len = std::min(static_cast<size_t>(bufsize > 1 ? bufsize - 1 : 0), kDs5SplitSecondChunkLen);
            std::memcpy(ds5_split_output_report + kDs5SplitFirstChunkLen, buffer + 1, copy_len);
            ds5_split_output_pending = false;
            handle_complete_ds5_output_report(ds5_split_output_report, kDs5OutputReportLen);
            return true;
        }
    }

    if (report_id == 0xF6) {
        if (buffer[0] == 0x20) {
            std::memset(ds5_feature_output_report, 0, sizeof(ds5_feature_output_report));
            const size_t copy_len = std::min(static_cast<size_t>(bufsize > 1 ? bufsize - 1 : 0), kDs5FeatureFirstChunkLen);
            std::memcpy(ds5_feature_output_report, buffer + 1, copy_len);
            ds5_feature_output_pending = true;
            return true;
        }
        if (buffer[0] == 0x21) {
            if (!ds5_feature_output_pending) {
                return true;
            }
            if (bufsize > 1) {
                ds5_feature_output_report[kDs5OutputReportLen - 1] = buffer[1];
            }
            ds5_feature_output_pending = false;
            handle_complete_ds5_output_report(ds5_feature_output_report, kDs5OutputReportLen);
            return true;
        }
    }

    return false;
}

} // namespace

bool apply_ns2pro_serial_input_if_available() {
    uint8_t payload[64];
    size_t payload_len = 0;
    if (!ns2pro_serial_bridge_poll_latest_input(payload, &payload_len)) {
        return false;
    }

    Ns2ProInputState state{};
    if (!ns2pro_parse_input_report(payload, payload_len, &state)) {
        return false;
    }

    uint8_t next_report[63];
    ns2pro_build_ds5_input_report(state, next_report, ++ns2pro_report_tick);
    critical_section_enter_blocking(&report_cs);
    memcpy(interrupt_in_data, next_report, sizeof(next_report));
    report_dirty = true;
    critical_section_exit(&report_cs);
    ns2pro_serial_active = true;
    ns2pro_last_seen_ms = to_ms_since_boot(get_absolute_time());

    pico_cmd_set_ns2pro_connected(true);
    pico_cmd_set_ns2pro_battery_percent(state.battery_percent);
    pico_cmd_set_input_owner_policy(BRIDGE_INPUT_OWNER_NS2PRO);
    if (get_config().controller_mode == 2) {
        pico_cmd_set_input_owner(BRIDGE_INPUT_OWNER_AUTO);
    }
    return true;
}

void update_ns2pro_ble_state_if_available() {
    uint8_t payload[63];
    size_t payload_len = 0;
    if (!ns2pro_ble_poll_latest_input(payload, &payload_len)) {
        return;
    }

    Ns2ProInputState state{};
    if (!ns2pro_parse_input_report(payload, payload_len, &state)) {
        return;
    }

    ns2pro_ble_latest_state = state;
    ns2pro_ble_state_valid = true;
    ns2pro_ble_state_pending = true;

    ns2pro_serial_active = true;
    ns2pro_last_seen_ms = to_ms_since_boot(get_absolute_time());
    pico_cmd_set_ns2pro_connected(true);
    pico_cmd_set_ns2pro_battery_percent(ns2pro_ble_latest_state.battery_percent);
    pico_cmd_set_input_owner_policy(BRIDGE_INPUT_OWNER_NS2PRO);
    if (get_config().controller_mode == 2) {
        pico_cmd_set_input_owner(BRIDGE_INPUT_OWNER_AUTO);
    }
}

void dispatch_ns2pro_ble_state_if_due() {
    if (!ns2pro_ble_connected() || !ns2pro_ble_state_valid || !ns2pro_ble_state_pending) {
        return;
    }

    const uint64_t now_us = time_us_64();
    if (ns2pro_ble_next_dispatch_us != 0 && now_us < ns2pro_ble_next_dispatch_us) {
        return;
    }

    uint8_t next_report[63];
    ns2pro_build_ds5_input_report(ns2pro_ble_latest_state, next_report, ++ns2pro_report_tick);
    critical_section_enter_blocking(&report_cs);
    std::memcpy(interrupt_in_data, next_report, sizeof(next_report));
    report_dirty = true;
    critical_section_exit(&report_cs);

    ns2pro_ble_state_pending = false;
    ns2pro_ble_next_dispatch_us = now_us + kNs2ProBleDispatchIntervalUs;
}

void interrupt_loop() {
    if (!tud_hid_ready()) return;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (should_stream_continuous_usb_reports(now_ms)) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    uint8_t safe_report[63];

    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");

            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void __not_in_flash_func(on_bt_data)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    if (channel == INTERRUPT && len > 2 && data[1] == 0x31) {
        if ((data[2] >> 1) & 1) {
            if (len >= 4) {
                mic_add_queue(data + 4, len - 4);
            }
            return;
        }
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }

        // Wake-on-PS must observe every BT input report regardless of polling
        // mode: the wake feature has its own state to maintain (button-byte
        // diff for edge detection) and short-circuiting it on non-2 polling
        // modes silently breaks wake while the host is suspended.
        wake_on_bt_input(data + 3, len - 3);
        #ifdef ENABLE_WAKE_HID
        ps_shortcut_tick(data + 3, len - 3);
        #endif
        pico_cmd_set_input_owner_policy(BRIDGE_INPUT_OWNER_DS5);

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
            ds5_apply_input_tuning(interrupt_in_data, sizeof(interrupt_in_data));
#if ENABLE_BATT_LED
            battery_led_note_report();
#endif
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        ds5_apply_input_tuning(interrupt_in_data, sizeof(interrupt_in_data));
        report_dirty = true;
        critical_section_exit(&report_cs);
#if ENABLE_BATT_LED
        battery_led_note_report();
#endif
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        if (reqlen >= 8) {
            memset(buffer, 0, 8);
            return 8;
        }
        return 0;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    if (is_pico_cmd(report_id)) {
        return pico_cmd_get(report_id, buffer, reqlen);
    }
    if (usb_bridge_is_manager_persona()) {
        return 0;
    }

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        const uint16_t payload_len = static_cast<uint16_t>(feature_data.size() - 1);
        const uint16_t copy_len = payload_len < reqlen ? payload_len : reqlen;
        memcpy(buffer, feature_data.data() + 1, copy_len);
    }

    return feature_data.empty() ? 0 : static_cast<uint16_t>(feature_data.size() - 1);
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex); // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue); // bAlternateSetting

    if (itf == 1) {
        printf("[AUDIO] Set interface Speaker to alternate setting %d\n", alt);
        spk_active = alt;
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        // Drop keyboard SET_REPORT (host LED state).
        return;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    if (try_handle_ds5_output_transport(report_id, buffer, bufsize)) {
        return;
    }

    if (is_pico_cmd(report_id)) {
#if ENABLE_VERBOSE
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n", buffer[0]);
#endif
        pico_cmd_set(report_id, buffer, bufsize);
        return;
    }
    if (usb_bridge_is_manager_persona()) {
        return;
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        set_feature_data(report_id, const_cast<uint8_t *>(buffer), bufsize);
        return;
    }
}

int main() {
#if SYS_CLOCK_KHZ != 150000
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
#endif

    board_init();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#if !ENABLE_SERIAL && !ENABLE_NS2PRO_SERIAL_BRIDGE
    sleep_ms(150);
#endif
    board_init_after_tusb();
#if ENABLE_SERIAL
    stdio_usb_init();
#endif
    ns2pro_serial_bridge_init();

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    config_load();
    pico_led_set(false);
    pico_led_restore_configured_state();

#if ENABLE_BATT_LED
    battery_led_init();
#endif

#if !ENABLE_SERIAL && !ENABLE_NS2PRO_SERIAL_BRIDGE
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        if (!get_config().disable_pico_led) {
            for (int i = 0; i < 6; i++) {
                if (i % 2 == 0) {
                    pico_led_set_override(true);
                } else {
                    pico_led_set_override(false);
                }
                sleep_ms(500);
            }
            pico_led_restore_configured_state();
        }
    } else {
        printf("Clean boot\n");
    }
#endif

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    wake_init();

    ns2pro_config_load();
    ns2pro_wired_bond_saved = get_ns2pro_config().ble_has_target != 0;
    pico_cmd_set_ns2pro_ble_has_bond(ns2pro_wired_bond_saved);
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
    ns2pro_ble_init();
    pico_cmd_set_input_owner(BRIDGE_INPUT_OWNER_AUTO);
    pico_cmd_set_input_owner_policy(BRIDGE_INPUT_OWNER_AUTO);
    pico_cmd_set_ds5_connected(false);
    pico_cmd_set_ns2pro_connected(false);
    pico_cmd_set_ns2pro_battery_percent(0);

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();
    state_init();
    ns2pro_rumble_init();

#if !ENABLE_SERIAL && !ENABLE_NS2PRO_SERIAL_BRIDGE
    watchdog_enable(1000, true);
#endif

    while (1) {
#if !ENABLE_SERIAL && !ENABLE_NS2PRO_SERIAL_BRIDGE
        watchdog_update();
#endif
        cyw43_arch_poll();
        tud_task();
        usb_bridge_task();
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        sync_pairing_services(now_ms);
        const bool service_ns2pro_runtime = should_service_ns2pro_runtime(now_ms);
        if (service_ns2pro_runtime) {
            ns2pro_serial_bridge_task();
            ns2pro_ble_task();
            if (ns2pro_rumble_should_stream_output()) {
                uint8_t rumble_report[64];
                if (ns2pro_rumble_build_output_report(rumble_report)) {
                    send_ns2pro_output_report(rumble_report, sizeof(rumble_report));
                }
            }
            const bool serial_ns2pro_updated = apply_ns2pro_serial_input_if_available();
            if (!serial_ns2pro_updated) {
                update_ns2pro_ble_state_if_available();
                dispatch_ns2pro_ble_state_if_due();
            }
            if (ns2pro_last_seen_ms != 0 && (now_ms - ns2pro_last_seen_ms) > kNs2ProInputActivityTimeoutMs) {
                ns2pro_serial_active = false;
                pico_cmd_set_ns2pro_connected(false);
                ns2pro_ble_state_valid = false;
                ns2pro_ble_state_pending = false;
                ns2pro_ble_next_dispatch_us = 0;
            }
        }
        static uint32_t next_bridge_status_refresh_ms = 0;
        if (next_bridge_status_refresh_ms == 0 ||
            now_ms >= next_bridge_status_refresh_ms) {
            pico_cmd_set_ds5_connected(bt_classic_connected());
            if (service_ns2pro_runtime) {
                Ns2ProRumbleDebug rumble_debug{};
                ns2pro_rumble_get_debug(&rumble_debug);
                pico_cmd_set_ns2pro_rumble_debug(rumble_debug);
                ns2pro_wired_bond_saved = get_ns2pro_config().ble_has_target != 0;
                pico_cmd_set_ns2pro_ble_has_bond(ns2pro_wired_bond_saved);
            }
            sync_usb_bridge_connection(now_ms);
            next_bridge_status_refresh_ms = now_ms + kBridgeStatusRefreshIntervalMs;
        }
        wake_task();
        audio_loop();
        interrupt_loop();
#if ENABLE_BATT_LED
        battery_led_tick();
#endif
        bt_inquiring_led();
    }
}
