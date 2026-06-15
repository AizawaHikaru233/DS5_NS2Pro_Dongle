//
// Created by awalol on 2026/5/4.
//

#include "cmd.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "bridge_manager_config.h"
#include "bt.h"
#include "config.h"
#include "device/usbd.h"
#include "ns2pro_ble.h"
#include "ns2pro_config.h"
#include "ns2pro_rumble.h"
#include "ns2pro_serial_bridge.h"
#include "ns2pro_translator.h"
#include "pico/time.h"
#include "usb.h"

namespace {

constexpr uint32_t kPrepareDualSenseRuntimeHoldMs = 30'000;

BridgeStatusPayload bridge_status = {
    .version = 3,
    .backend = BRIDGE_BACKEND_NS2PRO,
    .reserved2 = 0,
    .last_error = 0,
    .reserved4 = {0},
    .input_owner = BRIDGE_INPUT_OWNER_AUTO,
    .ds5_connected = 0,
    .ns2pro_connected = 0,
    .input_owner_policy = BRIDGE_INPUT_OWNER_AUTO,
    .ns2pro_battery_percent = 0,
    .signal_strength = 0,
    .ns2pro_ble_state = NS2PRO_BLE_STATE_IDLE,
    .ns2pro_ble_last_error = 0,
    .ns2pro_ble_has_bond = 0,
    .reserved33 = {0},
    .rumble_sequence = 0,
    .rumble_low = 0,
    .rumble_high = 0,
    .rumble_source = 0,
    .ble_sent_count = 0,
    .usb_queued_count = 0,
};

Ns2ProCommandDebug g_ns2pro_command_debug = {
    .last_command = 0,
    .self_test_count = 0,
    .self_test_low = 0,
    .self_test_high = 0,
};

uint8_t sanitize_input_owner(uint8_t owner) {
    if (owner == BRIDGE_INPUT_OWNER_DS5 || owner == BRIDGE_INPUT_OWNER_NS2PRO) {
        return owner;
    }
    return BRIDGE_INPUT_OWNER_AUTO;
}

void clear_ns2pro_bond_fields() {
    auto config = get_ns2pro_config();
    config.ble_has_target = 0;
    config.ble_address_type = 0;
    memset(config.ble_address, 0, sizeof(config.ble_address));
    set_ns2pro_config(config);
}

} // namespace

bool is_pico_cmd(uint8_t report_id) {
    if (report_id == 0xf6 ||
        report_id == 0xf7 ||
        report_id == 0xf8 ||
        report_id == 0xf9
    ) {
        return true;
    }
    return false;
}

uint16_t pico_cmd_get(uint8_t report_id, uint8_t *buffer, uint16_t reqlen) {
    if (report_id == 0xf7) {
        printf("[HID] Receive 0xf7 getting config\n");
        if (sizeof(BridgeManagerConfigBody) > reqlen) {
            printf("[Config] Warning: Config_body overflow\n");
        }
        return static_cast<uint16_t>(bridge_manager_config_get(buffer, reqlen));
    }
    if (report_id == 0xf8) {
        printf("[HID] Receive 0xf8 getting firmware version\n");
        const auto len = std::min(strlen(PICO_PROGRAM_VERSION_STRING), static_cast<size_t>(reqlen));
        memcpy(buffer, PICO_PROGRAM_VERSION_STRING, len);
        return len;
    }
    if (report_id == 0xf9) {
        int8_t rssi = 0;
        bt_get_signal_strength(&rssi);
        bridge_status.signal_strength = rssi;
        if (reqlen == 0) {
            return 0;
        }
        const auto len = std::min(sizeof(bridge_status), static_cast<size_t>(reqlen));
        memcpy(buffer, &bridge_status, len);
#if ENABLE_VERBOSE
        printf("[HID] 0xf9 bridge status backend=%u ble=%u bond=%u connected=%u rssi=%d\n",
               bridge_status.backend,
               bridge_status.ns2pro_ble_state,
               bridge_status.ns2pro_ble_has_bond,
               bridge_status.ns2pro_connected,
               rssi);
#endif
        return len;
    }
    return 0;
}

void pico_cmd_set(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize) {
    (void) report_id;
    if (bufsize == 0) {
        return;
    }

    pico_cmd_note_ns2pro_command(buffer[0], bufsize >= 2 ? buffer[1] : 0, bufsize >= 3 ? buffer[2] : 0);

    // 0x01 update config in variable
    // 0x02 write config to flash
    // 0x03 reconnect tinyusb device;
    if (buffer[0] == CMD_UPDATE_CONFIG) {
#if ENABLE_VERBOSE
        printf("[CMD] Enter config set func\n");
#endif
        bridge_manager_config_set(buffer + 1, bufsize - 1);
    }
    if (buffer[0] == CMD_SAVE_CONFIG) {
        printf("[CMD] Enter config save func\n");
        bridge_manager_config_save();
    }
    if (buffer[0] == CMD_RECONNECT_USB) {
        printf("[CMD] Enter tud reconnect func\n");
        usb_bridge_disconnect();
        sleep_ms(150);
        usb_bridge_connect();
    }
    if (buffer[0] == CMD_PREPARE_DUALSENSE_RUNTIME) {
        usb_bridge_hold_dualsense_for_ms(kPrepareDualSenseRuntimeHoldMs);
    }
    if (buffer[0] == CMD_SET_INPUT_OWNER) {
        pico_cmd_set_input_owner(bufsize >= 2 ? buffer[1] : BRIDGE_INPUT_OWNER_AUTO);
        pico_cmd_set_input_owner_policy(bufsize >= 2 ? buffer[1] : BRIDGE_INPUT_OWNER_AUTO);
    }
    if (buffer[0] == CMD_UPDATE_NS2PRO_CONFIG) {
        set_ns2pro_config(buffer + 1, bufsize - 1);
    }
    if (buffer[0] == CMD_SAVE_NS2PRO_CONFIG) {
        ns2pro_config_save();
    }
    if (buffer[0] == CMD_NS2PRO_RUMBLE_SELF_TEST) {
        const uint8_t low = bufsize >= 2 ? buffer[1] : 0;
        const uint8_t high = bufsize >= 3 ? buffer[2] : low;
        ns2pro_rumble_set_direct(low, high);
    }
    if (buffer[0] == CMD_NS2PRO_RUMBLE_QUAD_SELF_TEST) {
        const uint8_t left_high = bufsize >= 2 ? buffer[1] : 0;
        const uint8_t left_low = bufsize >= 3 ? buffer[2] : 0;
        const uint8_t right_high = bufsize >= 4 ? buffer[3] : 0;
        const uint8_t right_low = bufsize >= 5 ? buffer[4] : 0;
        ns2pro_rumble_set_direct_quad(left_high, left_low, right_high, right_low);
    }
    if (buffer[0] == CMD_NS2PRO_BLE_START_PAIRING) {
        ns2pro_ble_request_pairing();
    }
    if (buffer[0] == CMD_NS2PRO_BLE_CLEAR_BOND) {
        ns2pro_ble_clear_bond();
        clear_ns2pro_bond_fields();
        bridge_status.ns2pro_ble_has_bond = 0;
        bridge_status.ns2pro_ble_last_error = 0;
        bridge_status.ns2pro_ble_state = NS2PRO_BLE_STATE_IDLE;
        ns2pro_config_save();
    }
    if (buffer[0] == CMD_NS2PRO_CALIBRATE_STICK_CENTER) {
        pico_cmd_set_last_error(0x41);
        if (ns2pro_calibrate_stick_center_from_latest()) {
            pico_cmd_set_last_error(0);
        } else {
            pico_cmd_set_last_error(0x42);
        }
    }
}

void pico_cmd_set_last_error(uint8_t error) {
    bridge_status.last_error = error;
}

void pico_cmd_set_input_owner(uint8_t owner) {
    bridge_status.input_owner = sanitize_input_owner(owner);
}

void pico_cmd_set_input_owner_policy(uint8_t owner) {
    bridge_status.input_owner_policy = sanitize_input_owner(owner);
}

void pico_cmd_set_ds5_connected(bool connected) {
    bridge_status.ds5_connected = connected ? 1 : 0;
}

void pico_cmd_set_ns2pro_connected(bool connected) {
    bridge_status.ns2pro_connected = connected ? 1 : 0;
}

void pico_cmd_set_ns2pro_battery_percent(uint8_t percent) {
    bridge_status.ns2pro_battery_percent = percent;
}

void pico_cmd_set_ns2pro_ble_state(uint8_t state) {
    bridge_status.ns2pro_ble_state = state;
}

void pico_cmd_set_ns2pro_ble_last_error(uint8_t error) {
    bridge_status.ns2pro_ble_last_error = error;
}

void pico_cmd_set_ns2pro_ble_has_bond(bool has_bond) {
    bridge_status.ns2pro_ble_has_bond = has_bond ? 1 : 0;
}

void pico_cmd_set_ns2pro_ble_pairing_debug(uint8_t status, uint8_t reason) {
    bridge_status.reserved4[0] = status;
    bridge_status.reserved4[1] = reason;
}

void pico_cmd_set_ns2pro_ble_reencryption_status(uint8_t status) {
    bridge_status.reserved4[2] = status;
}

void pico_cmd_set_ns2pro_ble_disconnect_reason(uint8_t reason) {
    bridge_status.reserved4[3] = reason;
}

void pico_cmd_set_ns2pro_ble_gatt_debug(uint8_t att_status, uint8_t stage) {
    bridge_status.reserved4[4] = att_status;
    bridge_status.reserved4[5] = stage;
}

void pico_cmd_clear_ns2pro_ble_debug() {
    memset(bridge_status.reserved4, 0, sizeof(bridge_status.reserved4));
}

uint8_t pico_cmd_get_ns2pro_ble_state() {
    return bridge_status.ns2pro_ble_state;
}

uint8_t pico_cmd_get_ns2pro_ble_last_error() {
    return bridge_status.ns2pro_ble_last_error;
}

void pico_cmd_set_ns2pro_rumble_debug(const Ns2ProRumbleDebug &debug) {
    bridge_status.rumble_sequence = debug.last_sequence;
    bridge_status.rumble_low = debug.last_mixed_low;
    bridge_status.rumble_high = debug.last_mixed_high;
    bridge_status.rumble_source = debug.last_source;
    bridge_status.ble_sent_count = debug.ble_sent_count;
    bridge_status.usb_queued_count = debug.usb_queued_count;
    bridge_status.reserved33[0] = g_ns2pro_command_debug.last_command;
    bridge_status.reserved33[1] = g_ns2pro_command_debug.self_test_count;
    bridge_status.reserved33[2] = g_ns2pro_command_debug.self_test_low;
    bridge_status.reserved33[3] = g_ns2pro_command_debug.self_test_high;
    bridge_status.reserved33[4] = debug.input_count;
    bridge_status.reserved33[5] = debug.build_count;
    bridge_status.reserved33[6] = debug.send_attempt_count;
    bridge_status.reserved33[7] = debug.send_success_count;
    bridge_status.reserved33[8] = debug.send_failure_count;
    bridge_status.reserved33[9] = debug.last_send_stage;
    bridge_status.reserved33[10] = ns2pro_serial_bridge_last_output_status();
    memset(bridge_status.reserved33 + 11, 0, sizeof(bridge_status.reserved33) - 11);
    memcpy(bridge_status.reserved33 + 11, debug.last_report_head, sizeof(debug.last_report_head));
}

void pico_cmd_note_ns2pro_command(uint8_t command, uint8_t arg0, uint8_t arg1) {
    g_ns2pro_command_debug.last_command = command;
    if (command == CMD_NS2PRO_RUMBLE_SELF_TEST || command == CMD_NS2PRO_RUMBLE_QUAD_SELF_TEST) {
        g_ns2pro_command_debug.self_test_count =
            static_cast<uint8_t>(g_ns2pro_command_debug.self_test_count + 1u);
        g_ns2pro_command_debug.self_test_low = arg0;
        g_ns2pro_command_debug.self_test_high = arg1;
    }
}
