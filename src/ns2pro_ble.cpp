#include "ns2pro_ble.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

#include "ad_parser.h"
#include "bluetooth.h"
#include "bluetooth_gatt.h"
#include "bluetooth_data_types.h"
#include "ble/gatt_client.h"
#include "ble/sm.h"
#include "bt.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"
#include "cmd.h"
#include "gap.h"
#include "ns2pro_config.h"
#include "pico.h"
#include "pico/time.h"
#include "utils.h"

namespace {

constexpr uint16_t kNintendoCompanyId = 0x0553;
constexpr uint16_t kScanInterval = 0x0030;
constexpr uint16_t kScanWindow = 0x0030;
constexpr uint32_t kAlternateWindowMs = 2200;
constexpr uint32_t kIdleBleTaskIntervalMs = 40;
constexpr uint32_t kManualPairingWindowMs = 10000;
constexpr size_t kMaxInputPayload = 63;
constexpr uint16_t kFastConnIntervalMin = 6;
constexpr uint16_t kFastConnIntervalMax = 6;
constexpr uint16_t kFastConnLatency = 0;
constexpr uint16_t kFastConnSupervisionTimeout = 100;
constexpr uint32_t kHomeDisconnectHoldMs = 5000;
constexpr uint32_t kAutoReconnectHoldoffMs = 6000;
constexpr size_t kNs2ProOutputReportLen = 64;
constexpr size_t kNs2ProBleRumblePacketLen = 42;
constexpr size_t kNs2ProBleRumbleSideBlockLen = 16;

constexpr uint8_t kBleErrorScan = 0x01;
constexpr uint8_t kBleErrorConnect = 0x02;
constexpr uint8_t kBleErrorDiscover = 0x03;
constexpr uint8_t kBleErrorSubscribe = 0x04;
constexpr uint8_t kBleErrorInitWrite = 0x05;
constexpr uint8_t kBleErrorPairing = 0x06;

constexpr char kNotifyFd2UuidText[] = "ab7de9be-89fe-49ad-828f-118f09df7fd2";
constexpr char kAckUuidText[] = "c765a961-d9d8-4d36-a20a-5315b111836a";
constexpr char kCmdUuidText[] = "649d4ac9-8eb7-4e6c-af44-1ea54fe5f005";
constexpr char kRumbleUuidText[] = "cc483f51-9258-427d-a939-630c31f72b05";

struct InitCommand {
    const uint8_t *data;
    uint16_t len;
};

static const uint8_t kInitCmd0[] = {0x03, 0x91, 0x01, 0x0d, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const uint8_t kInitCmd1[] = {0x07, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd2[] = {0x16, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd3[] = {0x15, 0x91, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd4[] = {0x0c, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd5[] = {0x11, 0x91, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd6[] = {0x0a, 0x91, 0x01, 0x08, 0x00, 0x14, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x35, 0x00, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd7[] = {0x0c, 0x91, 0x01, 0x04, 0x00, 0x04, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd8[] = {0x03, 0x91, 0x01, 0x0a, 0x00, 0x04, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd9[] = {0x10, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd10[] = {0x01, 0x91, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd11[] = {0x01, 0x91, 0x01, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd12[] = {0x09, 0x91, 0x01, 0x07, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kInitCmd13[] = {0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x09, 0x7e, 0x00, 0x00, 0xa8, 0x30, 0x01, 0x00};
static const uint8_t kInitCmd14[] = {0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x09, 0x7e, 0x00, 0x00, 0xe8, 0x30, 0x01, 0x00};

static const InitCommand kInitCommands[] = {
    {kInitCmd0, sizeof(kInitCmd0)},
    {kInitCmd1, sizeof(kInitCmd1)},
    {kInitCmd2, sizeof(kInitCmd2)},
    {kInitCmd3, sizeof(kInitCmd3)},
    {kInitCmd4, sizeof(kInitCmd4)},
    {kInitCmd5, sizeof(kInitCmd5)},
    {kInitCmd6, sizeof(kInitCmd6)},
    {kInitCmd7, sizeof(kInitCmd7)},
    {kInitCmd8, sizeof(kInitCmd8)},
    {kInitCmd9, sizeof(kInitCmd9)},
    {kInitCmd10, sizeof(kInitCmd10)},
    {kInitCmd11, sizeof(kInitCmd11)},
    {kInitCmd12, sizeof(kInitCmd12)},
    {kInitCmd13, sizeof(kInitCmd13)},
    {kInitCmd14, sizeof(kInitCmd14)},
};

enum class DiscoverStage : uint8_t {
    None,
    FindFd2,
    FindAck,
    FindCmd,
    FindRumble,
    FindAckDescriptor,
    FindFd2Descriptor,
    SubscribeAck,
    WaitInitAck,
    SubscribeFd2,
    Ready,
};

btstack_packet_callback_registration_t g_hci_callback_registration{};
btstack_packet_callback_registration_t g_sm_callback_registration{};
gatt_client_notification_t g_ack_listener{};
gatt_client_notification_t g_fd2_listener{};
btstack_context_callback_registration_t g_write_without_response_request{};
btstack_context_callback_registration_t g_rumble_write_without_response_request{};

bool g_runtime_initialized = false;
bool g_stack_services_registered = false;
bool g_stack_ready = false;
bool g_pairing_service_enabled = true;
bool g_idle_pairing_enabled = true;
bool g_pairing_requested = false;
bool g_pairing_prefer_new_target = false;
bool g_alternating_scan = false;
bool g_classic_scan_slot = true;
bool g_ble_scan_active = false;
bool g_ble_connecting = false;
bool g_ble_connected = false;
bool g_input_stream_ready = false;
bool g_saved_target_valid = false;
bool g_active_target_valid = false;
bool g_home_button_down = false;
bool g_home_disconnect_sent = false;
bool g_cmd_write_without_response = false;
bool g_have_fd2 = false;
bool g_have_ack = false;
bool g_have_cmd = false;
bool g_have_rumble = false;
bool g_rumble_write_without_response = false;
bool g_latest_input_ready = false;
bool g_auto_reconnect_suppressed = false;
absolute_time_t g_next_idle_task_time{};
absolute_time_t g_manual_pairing_deadline{};
bd_addr_t g_saved_target_addr{};
bd_addr_t g_active_target_addr{};
bd_addr_type_t g_saved_target_addr_type = BD_ADDR_TYPE_LE_PUBLIC;
bd_addr_type_t g_active_target_addr_type = BD_ADDR_TYPE_LE_PUBLIC;
hci_con_handle_t g_conn_handle = HCI_CON_HANDLE_INVALID;
DiscoverStage g_discover_stage = DiscoverStage::None;
absolute_time_t g_last_slot_switch{};
uint32_t g_home_button_down_ms = 0;
uint32_t g_auto_reconnect_resume_ms = 0;
size_t g_init_index = 0;
uint8_t g_uuid_fd2[16]{};
uint8_t g_uuid_ack[16]{};
uint8_t g_uuid_cmd[16]{};
uint8_t g_uuid_rumble[16]{};
uint8_t g_latest_input[kMaxInputPayload]{};
uint8_t g_last_dispatched_input[kMaxInputPayload]{};
uint8_t g_subscribe_value[2]{};
size_t g_latest_input_len = 0;
size_t g_last_dispatched_input_len = 0;
gatt_client_characteristic_t g_chr_fd2{};
gatt_client_characteristic_t g_chr_ack{};
gatt_client_characteristic_t g_chr_cmd{};
gatt_client_characteristic_t g_chr_rumble{};
uint16_t g_chr_fd2_cccd_handle = 0;
uint16_t g_chr_ack_cccd_handle = 0;
uint8_t g_last_output_status = 0;
bool g_rumble_write_request_in_flight = false;
bool g_pending_rumble_valid = false;
uint8_t g_pending_rumble_packet[kNs2ProBleRumblePacketLen]{};

static void ns2pro_ble_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void on_query_complete(uint8_t att_status);
void request_next_init_command_write();
bool request_pending_rumble_write();
void maybe_start_ble_scan();

void request_fast_connection_parameters(const char *reason) {
    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    const int status = gap_request_connection_parameter_update(
        g_conn_handle,
        kFastConnIntervalMin,
        kFastConnIntervalMax,
        kFastConnLatency,
        kFastConnSupervisionTimeout
    );
    std::printf("[NS2PRO_BLE] fast conn update reason=%s handle=0x%04x interval=%u..%u latency=%u supervision=%u status=%d\n",
                reason ? reason : "unknown",
                g_conn_handle,
                static_cast<unsigned>(kFastConnIntervalMin),
                static_cast<unsigned>(kFastConnIntervalMax),
                static_cast<unsigned>(kFastConnLatency),
                static_cast<unsigned>(kFastConnSupervisionTimeout),
                status);
}

bool contains_ci(const char *haystack, const char *needle) {
    if (!haystack || !needle || !needle[0]) {
        return false;
    }
    const size_t needle_len = std::strlen(needle);
    for (const char *p = haystack; *p; ++p) {
        size_t i = 0;
        while (i < needle_len &&
               p[i] &&
               std::tolower(static_cast<unsigned char>(p[i])) ==
                   std::tolower(static_cast<unsigned char>(needle[i]))) {
            ++i;
        }
        if (i == needle_len) {
            return true;
        }
    }
    return false;
}

bool parse_hex_nibble(char c, uint8_t *value) {
    if (c >= '0' && c <= '9') {
        *value = static_cast<uint8_t>(c - '0');
        return true;
    }
    if (c >= 'a' && c <= 'f') {
        *value = static_cast<uint8_t>(10 + c - 'a');
        return true;
    }
    if (c >= 'A' && c <= 'F') {
        *value = static_cast<uint8_t>(10 + c - 'A');
        return true;
    }
    return false;
}

bool parse_uuid128(const char *text, uint8_t out[16]) {
    size_t index = 0;
    for (size_t i = 0; text[i] != 0;) {
        if (text[i] == '-') {
            ++i;
            continue;
        }
        if (text[i + 1] == 0 || index >= 16) {
            return false;
        }
        uint8_t hi = 0;
        uint8_t lo = 0;
        if (!parse_hex_nibble(text[i], &hi) || !parse_hex_nibble(text[i + 1], &lo)) {
            return false;
        }
        out[index++] = static_cast<uint8_t>((hi << 4) | lo);
        i += 2;
    }
    return index == 16;
}

void clear_characteristics() {
    std::memset(&g_chr_fd2, 0, sizeof(g_chr_fd2));
    std::memset(&g_chr_ack, 0, sizeof(g_chr_ack));
    std::memset(&g_chr_cmd, 0, sizeof(g_chr_cmd));
    std::memset(&g_chr_rumble, 0, sizeof(g_chr_rumble));
    g_have_fd2 = false;
    g_have_ack = false;
    g_have_cmd = false;
    g_have_rumble = false;
    g_cmd_write_without_response = false;
    g_rumble_write_without_response = false;
    g_input_stream_ready = false;
    g_chr_fd2_cccd_handle = 0;
    g_chr_ack_cccd_handle = 0;
    g_rumble_write_request_in_flight = false;
    g_pending_rumble_valid = false;
    g_last_output_status = 0;
}

bool build_rumble_packet_from_output_report(const uint8_t *report,
                                            size_t len,
                                            uint8_t packet[kNs2ProBleRumblePacketLen]) {
    if (!report || len != kNs2ProOutputReportLen || report[0] != 0x02) {
        return false;
    }

    std::memset(packet, 0, kNs2ProBleRumblePacketLen);
    std::memcpy(packet + 1, report + 1, kNs2ProBleRumbleSideBlockLen);
    std::memcpy(packet + 17, report + 17, kNs2ProBleRumbleSideBlockLen);
    return true;
}

void reset_latest_input() {
    g_latest_input_ready = false;
    g_latest_input_len = 0;
    g_last_dispatched_input_len = 0;
    std::memset(g_latest_input, 0, sizeof(g_latest_input));
    std::memset(g_last_dispatched_input, 0, sizeof(g_last_dispatched_input));
}

bool is_duplicate_input_sample(const uint8_t *payload, size_t len) {
    if (!payload) {
        return false;
    }
    if (g_latest_input_ready &&
        len == g_latest_input_len &&
        std::memcmp(payload, g_latest_input, len) == 0) {
        return true;
    }
    if (len == g_last_dispatched_input_len &&
        std::memcmp(payload, g_last_dispatched_input, len) == 0) {
        return true;
    }
    return false;
}

void load_saved_target_from_config() {
    const auto &cfg = get_ns2pro_config();
    g_saved_target_valid = cfg.ble_has_target != 0;
    if (!g_saved_target_valid) {
        std::memset(g_saved_target_addr, 0, sizeof(g_saved_target_addr));
        g_saved_target_addr_type = BD_ADDR_TYPE_LE_PUBLIC;
        return;
    }
    std::memcpy(g_saved_target_addr, cfg.ble_address, sizeof(g_saved_target_addr));
    g_saved_target_addr_type = static_cast<bd_addr_type_t>(cfg.ble_address_type);
}

void save_active_target_to_config() {
    if (!g_active_target_valid) {
        return;
    }
    auto cfg = get_ns2pro_config();
    cfg.ble_has_target = 1;
    cfg.ble_address_type = static_cast<uint8_t>(g_active_target_addr_type);
    std::memcpy(cfg.ble_address, g_active_target_addr, sizeof(cfg.ble_address));
    set_ns2pro_config(cfg);
    ns2pro_config_save();
    g_saved_target_valid = true;
    g_saved_target_addr_type = g_active_target_addr_type;
    std::memcpy(g_saved_target_addr, g_active_target_addr, sizeof(g_saved_target_addr));
    pico_cmd_set_ns2pro_ble_has_bond(true);
}

void delete_saved_target_bonding() {
    if (!g_saved_target_valid) {
        return;
    }
    std::printf("[NS2PRO_BLE] delete saved LE bond addr=%s type=%u\n",
                bd_addr_to_str(g_saved_target_addr),
                static_cast<unsigned>(g_saved_target_addr_type));
    gap_delete_bonding(g_saved_target_addr_type, g_saved_target_addr);
}

void delete_active_target_bonding() {
    if (!g_active_target_valid) {
        return;
    }
    std::printf("[NS2PRO_BLE] delete active LE bond addr=%s type=%u\n",
                bd_addr_to_str(g_active_target_addr),
                static_cast<unsigned>(g_active_target_addr_type));
    gap_delete_bonding(g_active_target_addr_type, g_active_target_addr);
}

void set_ble_error(uint8_t error) {
    pico_cmd_set_ns2pro_ble_last_error(error);
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_ERROR);
}

void stop_ble_scan() {
    if (!g_ble_scan_active) {
        return;
    }
    gap_stop_scan();
    g_ble_scan_active = false;
}

void stop_alternating_scan(bool classic_enabled_after_stop) {
    g_pairing_requested = false;
    g_pairing_prefer_new_target = false;
    g_alternating_scan = false;
    stop_ble_scan();
    bt_set_classic_pairing_enabled(classic_enabled_after_stop && bt_classic_pairing_service_enabled());
}

void stop_all_scans_for_success() {
    stop_alternating_scan(false);
}

void reset_home_disconnect_tracking() {
    g_home_button_down = false;
    g_home_disconnect_sent = false;
    g_home_button_down_ms = 0;
}

void update_home_disconnect_state(const uint8_t *value, uint16_t value_length) {
    if (!value || value_length <= 5 || !g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        reset_home_disconnect_tracking();
        return;
    }

    const bool home_pressed = (value[5] & 0x10) != 0;
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if (!home_pressed) {
        reset_home_disconnect_tracking();
        return;
    }

    if (!g_home_button_down) {
        g_home_button_down = true;
        g_home_button_down_ms = now_ms;
        g_home_disconnect_sent = false;
        return;
    }

    if (!g_home_disconnect_sent &&
        now_ms - g_home_button_down_ms >= kHomeDisconnectHoldMs) {
        g_home_disconnect_sent = true;
        g_auto_reconnect_suppressed = true;
        g_auto_reconnect_resume_ms = now_ms + kAutoReconnectHoldoffMs;
        std::printf("[NS2PRO_BLE] home held for %u ms, disconnecting handle=0x%04x\n",
                    static_cast<unsigned>(kHomeDisconnectHoldMs),
                    g_conn_handle);
        gap_disconnect(g_conn_handle);
    }
}

void begin_scan_session(bool prefer_new_target, bool alternate_with_classic, bool start_with_classic_slot) {
    g_pairing_requested = true;
    g_pairing_prefer_new_target = prefer_new_target;
    g_alternating_scan = alternate_with_classic;
    g_classic_scan_slot = start_with_classic_slot;
    g_last_slot_switch = get_absolute_time();

    stop_ble_scan();
    if (g_alternating_scan) {
        bt_set_classic_pairing_enabled(g_classic_scan_slot);
    } else {
        bt_set_classic_pairing_enabled(false);
    }
    pico_cmd_set_ns2pro_ble_state(
        prefer_new_target ? NS2PRO_BLE_STATE_PAIRING_REQUESTED : NS2PRO_BLE_STATE_SCANNING
    );
    maybe_start_ble_scan();
}

void copy_adv_name(uint8_t adv_len, const uint8_t *adv_data, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
        const uint8_t data_type = ad_iterator_get_data_type(&context);
        if (data_type != BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME &&
            data_type != BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME) {
            continue;
        }
        const uint8_t data_size = ad_iterator_get_data_len(&context);
        const uint8_t *data = ad_iterator_get_data(&context);
        const size_t copy_len = std::min(static_cast<size_t>(data_size), out_len - 1);
        std::memcpy(out, data, copy_len);
        out[copy_len] = 0;
        return;
    }
}

bool adv_has_company_id(uint8_t adv_len, const uint8_t *adv_data, uint16_t company_id) {
    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
        const uint8_t data_type = ad_iterator_get_data_type(&context);
        const uint8_t data_size = ad_iterator_get_data_len(&context);
        const uint8_t *data = ad_iterator_get_data(&context);
        if (data_type != BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA || data_size < 2) {
            continue;
        }
        const uint16_t found = static_cast<uint16_t>(data[0] | (data[1] << 8));
        if (found == company_id) {
            return true;
        }
    }
    return false;
}

bool adv_is_candidate(const char *name, bool nintendo_mfg) {
    return nintendo_mfg ||
           contains_ci(name, "switch") ||
           contains_ci(name, "nintendo") ||
           contains_ci(name, "pro controller") ||
           contains_ci(name, "pro2");
}

void start_next_discovery_step() {
    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    const uint8_t *uuid = nullptr;
    switch (g_discover_stage) {
        case DiscoverStage::FindFd2:
            uuid = g_uuid_fd2;
            break;
        case DiscoverStage::FindAck:
            uuid = g_uuid_ack;
            break;
        case DiscoverStage::FindCmd:
            uuid = g_uuid_cmd;
            break;
        case DiscoverStage::FindRumble:
            uuid = g_uuid_rumble;
            break;
        default:
            return;
    }

    const uint8_t status = gatt_client_discover_characteristics_for_handle_range_by_uuid128(
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        0x0001,
        0xffff,
        uuid
    );
    if (status != ERROR_CODE_SUCCESS) {
        std::printf("[NS2PRO_BLE] characteristic discovery start failed stage=%u status=0x%02x\n",
                    static_cast<unsigned>(g_discover_stage), status);
        set_ble_error(kBleErrorDiscover);
    }
}

void remember_found_characteristic(const gatt_client_characteristic_t &characteristic) {
    switch (g_discover_stage) {
        case DiscoverStage::FindFd2:
            g_chr_fd2 = characteristic;
            g_have_fd2 = true;
            break;
        case DiscoverStage::FindAck:
            g_chr_ack = characteristic;
            g_have_ack = true;
            break;
        case DiscoverStage::FindCmd:
            g_chr_cmd = characteristic;
            g_have_cmd = true;
            g_cmd_write_without_response =
                (characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            break;
        case DiscoverStage::FindRumble:
            g_chr_rumble = characteristic;
            g_have_rumble = true;
            g_rumble_write_without_response =
                (characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            break;
        default:
            break;
    }
}

gatt_client_characteristic_t *characteristic_for_descriptor_stage() {
    switch (g_discover_stage) {
        case DiscoverStage::FindAckDescriptor:
            return &g_chr_ack;
        case DiscoverStage::FindFd2Descriptor:
            return &g_chr_fd2;
        default:
            return nullptr;
    }
}

uint16_t *cccd_handle_for_descriptor_stage() {
    switch (g_discover_stage) {
        case DiscoverStage::FindAckDescriptor:
            return &g_chr_ack_cccd_handle;
        case DiscoverStage::FindFd2Descriptor:
            return &g_chr_fd2_cccd_handle;
        default:
            return nullptr;
    }
}

const char *label_for_descriptor_stage() {
    switch (g_discover_stage) {
        case DiscoverStage::FindAckDescriptor:
            return "ack";
        case DiscoverStage::FindFd2Descriptor:
            return "fd2";
        default:
            return "unknown";
    }
}

void start_current_descriptor_discovery() {
    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    gatt_client_characteristic_t *characteristic = characteristic_for_descriptor_stage();
    if (!characteristic) {
        return;
    }
    if (characteristic->end_handle <= characteristic->value_handle) {
        std::printf("[NS2PRO_BLE] %s descriptor discovery skipped value=0x%04x end=0x%04x\n",
                    label_for_descriptor_stage(),
                    characteristic->value_handle,
                    characteristic->end_handle);
        on_query_complete(ATT_ERROR_SUCCESS);
        return;
    }

    const uint8_t status = gatt_client_discover_characteristic_descriptors(
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        characteristic
    );
    if (status != ERROR_CODE_SUCCESS) {
        std::printf("[NS2PRO_BLE] %s descriptor discovery start failed value=0x%04x end=0x%04x status=0x%02x\n",
                    label_for_descriptor_stage(),
                    characteristic->value_handle,
                    characteristic->end_handle,
                    status);
        set_ble_error(kBleErrorDiscover);
        return;
    }

    std::printf("[NS2PRO_BLE] %s descriptor discovery start value=0x%04x end=0x%04x\n",
                label_for_descriptor_stage(),
                characteristic->value_handle,
                characteristic->end_handle);
}

void remember_found_descriptor(const gatt_client_characteristic_descriptor_t &descriptor) {
    uint16_t *cccd_handle = cccd_handle_for_descriptor_stage();
    if (!cccd_handle) {
        return;
    }

    std::printf("[NS2PRO_BLE] %s descriptor handle=0x%04x uuid16=0x%04x\n",
                label_for_descriptor_stage(),
                descriptor.handle,
                descriptor.uuid16);
    if (descriptor.uuid16 == ORG_BLUETOOTH_DESCRIPTOR_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION) {
        *cccd_handle = descriptor.handle;
    }
}

uint16_t cccd_mode_for_characteristic(const gatt_client_characteristic_t &characteristic) {
    if ((characteristic.properties & ATT_PROPERTY_INDICATE) != 0) {
        return GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_INDICATION;
    }
    if ((characteristic.properties & ATT_PROPERTY_NOTIFY) != 0) {
        return GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    }
    return 0;
}

bool subscribe_to_characteristic(gatt_client_notification_t *listener,
                                 gatt_client_characteristic_t *characteristic,
                                 uint16_t explicit_cccd_handle,
                                 const char *label) {
    const uint16_t cccd_mode = cccd_mode_for_characteristic(*characteristic);
    if (cccd_mode == 0) {
        std::printf("[NS2PRO_BLE] %s has no notify/indicate property handle=0x%04x props=0x%02x\n",
                    label,
                    characteristic->value_handle,
                    characteristic->properties);
        set_ble_error(kBleErrorSubscribe);
        return false;
    }

    gatt_client_listen_for_characteristic_value_updates(
        listener,
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        characteristic
    );

    const uint16_t cccd_handle =
        explicit_cccd_handle != 0 ? explicit_cccd_handle : static_cast<uint16_t>(characteristic->value_handle + 1);
    g_subscribe_value[0] =
        (cccd_mode == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_INDICATION) ? 0x02 : 0x01;
    g_subscribe_value[1] = 0x00;

    const uint8_t status = gatt_client_write_characteristic_descriptor_using_descriptor_handle(
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        cccd_handle,
        sizeof(g_subscribe_value),
        g_subscribe_value
    );
    if (status != ERROR_CODE_SUCCESS) {
        std::printf("[NS2PRO_BLE] %s subscribe start failed handle=0x%04x props=0x%02x cccd=0x%04x value=%u status=0x%02x\n",
                    label,
                    characteristic->value_handle,
                    characteristic->properties,
                    cccd_handle,
                    static_cast<unsigned>(g_subscribe_value[0]),
                    status);
        set_ble_error(kBleErrorSubscribe);
        return false;
    }

    std::printf("[NS2PRO_BLE] %s subscribe start handle=0x%04x props=0x%02x cccd=0x%04x value=%u\n",
                label,
                characteristic->value_handle,
                characteristic->properties,
                cccd_handle,
                static_cast<unsigned>(g_subscribe_value[0]));
    return true;
}

void write_without_response_ready(void *context) {
    (void)context;
    if (!g_ble_connected || !g_have_cmd || g_init_index >= std::size(kInitCommands)) {
        return;
    }

    const InitCommand &cmd = kInitCommands[g_init_index];
    const uint8_t status = gatt_client_write_value_of_characteristic_without_response(
        g_conn_handle,
        g_chr_cmd.value_handle,
        cmd.len,
        const_cast<uint8_t *>(cmd.data)
    );
    if (status != ERROR_CODE_SUCCESS) {
        std::printf("[NS2PRO_BLE] init write no-rsp failed status=0x%02x index=%u\n",
                    status, static_cast<unsigned>(g_init_index));
        set_ble_error(kBleErrorInitWrite);
    }
}

void rumble_write_without_response_ready(void *context) {
    (void)context;
    g_rumble_write_request_in_flight = false;

    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        g_last_output_status = 2;
        return;
    }
    if (!g_have_rumble || g_chr_rumble.value_handle == 0) {
        g_last_output_status = 3;
        return;
    }
    if (!g_pending_rumble_valid) {
        g_last_output_status = 8;
        return;
    }

    uint8_t packet[kNs2ProBleRumblePacketLen];
    std::memcpy(packet, g_pending_rumble_packet, sizeof(packet));
    g_pending_rumble_valid = false;

    const uint8_t status = gatt_client_write_value_of_characteristic_without_response(
        g_conn_handle,
        g_chr_rumble.value_handle,
        sizeof(packet),
        packet
    );
    if (status != ERROR_CODE_SUCCESS) {
        g_last_output_status = static_cast<uint8_t>(0x80u | status);
        return;
    }

    g_last_output_status = 7;

    if (g_pending_rumble_valid) {
        request_pending_rumble_write();
    }
}

void request_next_init_command_write() {
    if (!g_ble_connected || !g_have_cmd || g_init_index >= std::size(kInitCommands)) {
        return;
    }

    const InitCommand &cmd = kInitCommands[g_init_index];
    if (g_cmd_write_without_response) {
        g_write_without_response_request.callback = &write_without_response_ready;
        g_write_without_response_request.context = nullptr;
        const uint8_t status = gatt_client_request_to_write_without_response(
            &g_write_without_response_request,
            g_conn_handle
        );
        if (status != ERROR_CODE_SUCCESS) {
            std::printf("[NS2PRO_BLE] request can-write failed status=0x%02x index=%u\n",
                        status, static_cast<unsigned>(g_init_index));
            set_ble_error(kBleErrorInitWrite);
        }
        return;
    }

    const uint8_t status = gatt_client_write_value_of_characteristic(
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        g_chr_cmd.value_handle,
        cmd.len,
        const_cast<uint8_t *>(cmd.data)
    );
    if (status != ERROR_CODE_SUCCESS) {
        std::printf("[NS2PRO_BLE] init write failed status=0x%02x index=%u\n",
                    status, static_cast<unsigned>(g_init_index));
        set_ble_error(kBleErrorInitWrite);
    }
}

bool request_pending_rumble_write() {
    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        g_last_output_status = 2;
        return false;
    }
    if (!g_have_rumble || g_chr_rumble.value_handle == 0) {
        g_last_output_status = 3;
        return false;
    }
    if (!g_pending_rumble_valid) {
        g_last_output_status = 8;
        return false;
    }
    if (g_rumble_write_request_in_flight) {
        g_last_output_status = 5;
        return true;
    }

    g_rumble_write_without_response_request.callback = &rumble_write_without_response_ready;
    g_rumble_write_without_response_request.context = nullptr;
    g_rumble_write_request_in_flight = true;
    const uint8_t status = gatt_client_request_to_write_without_response(
        &g_rumble_write_without_response_request,
        g_conn_handle
    );
    if (status != ERROR_CODE_SUCCESS) {
        g_rumble_write_request_in_flight = false;
        g_last_output_status = static_cast<uint8_t>(0x90u | status);
        return false;
    }

    g_last_output_status = 5;
    return true;
}

void on_query_complete(uint8_t att_status) {
    pico_cmd_set_ns2pro_ble_gatt_debug(att_status, static_cast<uint8_t>(g_discover_stage));
    if (att_status != ATT_ERROR_SUCCESS) {
        std::printf("[NS2PRO_BLE] query complete error stage=%u att=0x%02x\n",
                    static_cast<unsigned>(g_discover_stage), att_status);
        if (g_discover_stage == DiscoverStage::SubscribeAck ||
            g_discover_stage == DiscoverStage::SubscribeFd2) {
            set_ble_error(kBleErrorSubscribe);
        } else if (g_discover_stage == DiscoverStage::WaitInitAck) {
            set_ble_error(kBleErrorInitWrite);
        } else {
            set_ble_error(kBleErrorDiscover);
        }
        return;
    }

    switch (g_discover_stage) {
        case DiscoverStage::FindFd2:
            g_discover_stage = DiscoverStage::FindAck;
            start_next_discovery_step();
            break;
        case DiscoverStage::FindAck:
            g_discover_stage = DiscoverStage::FindCmd;
            start_next_discovery_step();
            break;
        case DiscoverStage::FindCmd:
            g_discover_stage = DiscoverStage::FindRumble;
            start_next_discovery_step();
            break;
        case DiscoverStage::FindRumble:
            if (!g_have_ack || !g_have_cmd) {
                set_ble_error(kBleErrorDiscover);
                break;
            }
            g_discover_stage = DiscoverStage::FindAckDescriptor;
            start_current_descriptor_discovery();
            break;
        case DiscoverStage::FindAckDescriptor:
            if (g_have_fd2) {
                g_discover_stage = DiscoverStage::FindFd2Descriptor;
                start_current_descriptor_discovery();
                break;
            }
            g_discover_stage = DiscoverStage::SubscribeAck;
            if (!subscribe_to_characteristic(&g_ack_listener, &g_chr_ack, g_chr_ack_cccd_handle, "ack")) {
                break;
            }
            break;
        case DiscoverStage::FindFd2Descriptor:
            g_discover_stage = DiscoverStage::SubscribeAck;
            if (!subscribe_to_characteristic(&g_ack_listener, &g_chr_ack, g_chr_ack_cccd_handle, "ack")) {
                break;
            }
            break;
        case DiscoverStage::SubscribeAck:
            g_discover_stage = DiscoverStage::WaitInitAck;
            g_init_index = 0;
            request_next_init_command_write();
            break;
        case DiscoverStage::WaitInitAck:
            break;
        case DiscoverStage::SubscribeFd2:
            break;
        case DiscoverStage::Ready:
        case DiscoverStage::None:
            break;
    }
}

void advance_init_from_ack() {
    if (g_discover_stage != DiscoverStage::WaitInitAck) {
        return;
    }
    ++g_init_index;
    if (g_init_index >= std::size(kInitCommands)) {
        if (!g_have_fd2) {
            set_ble_error(kBleErrorDiscover);
            return;
        }
        g_discover_stage = DiscoverStage::SubscribeFd2;
        if (!subscribe_to_characteristic(&g_fd2_listener, &g_chr_fd2, g_chr_fd2_cccd_handle, "fd2")) {
            return;
        }
        pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_INITIALIZING);
        return;
    }
    request_next_init_command_write();
}

void connect_to_candidate(bd_addr_t address, bd_addr_type_t address_type) {
    stop_ble_scan();
    g_ble_connecting = true;
    g_active_target_valid = true;
    g_active_target_addr_type = address_type;
    std::memcpy(g_active_target_addr, address, sizeof(g_active_target_addr));
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_CONNECTING);
    gap_connect(address, address_type);
}

void maybe_start_ble_scan() {
    if (!g_runtime_initialized || !g_stack_ready || !g_pairing_requested || g_ble_connected || g_ble_connecting || g_ble_scan_active) {
        return;
    }
    if (g_alternating_scan && g_classic_scan_slot) {
        return;
    }
    gap_set_scan_parameters(1, kScanInterval, kScanWindow);
    gap_start_scan();
    g_ble_scan_active = true;
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_SCANNING);
}

void on_le_connection_complete(uint8_t *packet) {
    const uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
    g_ble_connecting = false;
    if (status != ERROR_CODE_SUCCESS) {
        set_ble_error(kBleErrorConnect);
        return;
    }

    g_conn_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
    gap_subevent_le_connection_complete_get_peer_address(packet, g_active_target_addr);
    g_active_target_addr_type = static_cast<bd_addr_type_t>(
        gap_subevent_le_connection_complete_get_peer_address_type(packet));
    g_active_target_valid = true;
    g_ble_connected = true;
    reset_home_disconnect_tracking();
    std::printf("[NS2PRO_BLE] connected handle=0x%04x interval=%u latency=%u supervision=%u\n",
                g_conn_handle,
                static_cast<unsigned>(gap_subevent_le_connection_complete_get_conn_interval(packet)),
                static_cast<unsigned>(gap_subevent_le_connection_complete_get_conn_latency(packet)),
                static_cast<unsigned>(gap_subevent_le_connection_complete_get_supervision_timeout(packet)));
    stop_all_scans_for_success();
    ns2pro_ble_set_pairing_service_enabled(false);
    bt_set_classic_pairing_service_enabled(false);
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_INITIALIZING);
    clear_characteristics();
    g_discover_stage = DiscoverStage::FindFd2;
    g_init_index = 0;
    request_fast_connection_parameters("connect");
    start_next_discovery_step();
}

void on_adv_report(uint8_t *packet) {
    bd_addr_t address;
    gap_event_advertising_report_get_address(packet, address);
    const bd_addr_type_t address_type = static_cast<bd_addr_type_t>(
        gap_event_advertising_report_get_address_type(packet));
    const uint8_t adv_len = gap_event_advertising_report_get_data_length(packet);
    const uint8_t *adv_data = gap_event_advertising_report_get_data(packet);
    char name[32];
    copy_adv_name(adv_len, adv_data, name, sizeof(name));
    const bool nintendo_mfg = adv_has_company_id(adv_len, adv_data, kNintendoCompanyId);
    if (!adv_is_candidate(name, nintendo_mfg)) {
        return;
    }

    if (!g_pairing_prefer_new_target &&
        g_saved_target_valid &&
        (address_type != g_saved_target_addr_type ||
         std::memcmp(address, g_saved_target_addr, sizeof(g_saved_target_addr)) != 0)) {
        return;
    }

    std::printf("[NS2PRO_BLE] candidate %s addr=%s type=%u\n",
                name[0] ? name : "<unnamed>",
                bd_addr_to_str(address),
                static_cast<unsigned>(address_type));
    connect_to_candidate(address, address_type);
}

void on_le_disconnect(uint8_t *packet) {
    const hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
    if (handle != g_conn_handle) {
        return;
    }
    pico_cmd_set_ns2pro_ble_disconnect_reason(hci_event_disconnection_complete_get_reason(packet));
    g_conn_handle = HCI_CON_HANDLE_INVALID;
    g_ble_connected = false;
    g_ble_connecting = false;
    reset_home_disconnect_tracking();
    clear_characteristics();
    reset_latest_input();
    pico_cmd_set_ns2pro_connected(false);
    pico_cmd_set_ns2pro_battery_percent(0);
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
    if (!bt_classic_connected()) {
        ns2pro_ble_set_pairing_service_enabled(true);
        bt_set_classic_pairing_service_enabled(true);
        bt_set_classic_pairing_enabled(true);
    }
}

static void ns2pro_ble_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                g_stack_ready = true;
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            on_adv_report(packet);
            break;
        case HCI_EVENT_META_GAP:
            if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
                on_le_connection_complete(packet);
            }
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE &&
                hci_subevent_le_connection_update_complete_get_connection_handle(packet) == g_conn_handle) {
                std::printf("[NS2PRO_BLE] conn update complete status=0x%02x interval=%u latency=%u supervision=%u\n",
                            hci_subevent_le_connection_update_complete_get_status(packet),
                            static_cast<unsigned>(hci_subevent_le_connection_update_complete_get_conn_interval(packet)),
                            static_cast<unsigned>(hci_subevent_le_connection_update_complete_get_conn_latency(packet)),
                            static_cast<unsigned>(hci_subevent_le_connection_update_complete_get_supervision_timeout(packet)));
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            on_le_disconnect(packet);
            break;
        case SM_EVENT_JUST_WORKS_REQUEST:
            if (sm_event_just_works_request_get_handle(packet) == g_conn_handle) {
                sm_just_works_confirm(g_conn_handle);
            }
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            if (sm_event_pairing_complete_get_handle(packet) == g_conn_handle) {
                const uint8_t status = sm_event_pairing_complete_get_status(packet);
                const uint8_t reason = sm_event_pairing_complete_get_reason(packet);
                pico_cmd_set_ns2pro_ble_pairing_debug(
                    status,
                    reason
                );
                if (status != ERROR_CODE_SUCCESS &&
                    reason != SM_REASON_PAIRING_NOT_SUPPORTED) {
                    set_ble_error(kBleErrorPairing);
                }
            }
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            if (sm_event_reencryption_complete_get_handle(packet) == g_conn_handle) {
                const uint8_t status = sm_event_reencryption_complete_get_status(packet);
                pico_cmd_set_ns2pro_ble_reencryption_status(status);
                if (status != ERROR_CODE_SUCCESS &&
                    status != ERROR_CODE_PIN_OR_KEY_MISSING) {
                    set_ble_error(kBleErrorPairing);
                }
            }
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic{};
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            remember_found_characteristic(characteristic);
            break;
        }
        case GATT_EVENT_ALL_CHARACTERISTIC_DESCRIPTORS_QUERY_RESULT: {
            gatt_client_characteristic_descriptor_t descriptor{};
            gatt_event_all_characteristic_descriptors_query_result_get_characteristic_descriptor(packet, &descriptor);
            remember_found_descriptor(descriptor);
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            on_query_complete(gatt_event_query_complete_get_att_status(packet));
            break;
        case GATT_EVENT_NOTIFICATION:
        case GATT_EVENT_INDICATION: {
            const bool is_indication = hci_event_packet_get_type(packet) == GATT_EVENT_INDICATION;
            const uint16_t value_handle = is_indication
                                              ? gatt_event_indication_get_value_handle(packet)
                                              : gatt_event_notification_get_value_handle(packet);
            const uint16_t value_length = is_indication
                                              ? gatt_event_indication_get_value_length(packet)
                                              : gatt_event_notification_get_value_length(packet);
            const uint8_t *value = is_indication
                                       ? gatt_event_indication_get_value(packet)
                                       : gatt_event_notification_get_value(packet);
            if (g_have_ack && value_handle == g_chr_ack.value_handle) {
                advance_init_from_ack();
                break;
            }
            if (g_have_fd2 && value_handle == g_chr_fd2.value_handle) {
                update_home_disconnect_state(value, value_length);
                const size_t copy_len = std::min(static_cast<size_t>(value_length), sizeof(g_latest_input));
                if (!is_duplicate_input_sample(value, copy_len)) {
                    std::memcpy(g_latest_input, value, copy_len);
                    g_latest_input_len = copy_len;
                    g_latest_input_ready = true;
                }
                pico_cmd_set_ns2pro_connected(true);
                if (!g_input_stream_ready || g_discover_stage != DiscoverStage::Ready) {
                    g_input_stream_ready = true;
                    g_discover_stage = DiscoverStage::Ready;
                    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_READY);
                    save_active_target_to_config();
                    request_fast_connection_parameters("ready");
                }
            }
            break;
        }
        default:
            break;
    }
}

} // namespace

void ns2pro_ble_init() {
    g_runtime_initialized = true;
    g_stack_ready = bt_stack_ready();
    g_pairing_service_enabled = true;
    g_pairing_requested = false;
    g_pairing_prefer_new_target = false;
    g_alternating_scan = false;
    g_classic_scan_slot = true;
    g_ble_scan_active = false;
    g_ble_connecting = false;
    g_ble_connected = false;
    g_input_stream_ready = false;
    g_active_target_valid = false;
    g_conn_handle = HCI_CON_HANDLE_INVALID;
    g_discover_stage = DiscoverStage::None;
    g_init_index = 0;
    g_auto_reconnect_suppressed = false;
    g_auto_reconnect_resume_ms = 0;
    g_next_idle_task_time = nil_time;
    reset_home_disconnect_tracking();
    clear_characteristics();
    reset_latest_input();
    load_saved_target_from_config();
}

void ns2pro_ble_stack_init() {
    if (g_stack_services_registered) {
        g_stack_ready = g_stack_ready || bt_stack_ready();
        return;
    }

    if (g_uuid_fd2[0] == 0) {
        parse_uuid128(kNotifyFd2UuidText, g_uuid_fd2);
        parse_uuid128(kAckUuidText, g_uuid_ack);
        parse_uuid128(kCmdUuidText, g_uuid_cmd);
        parse_uuid128(kRumbleUuidText, g_uuid_rumble);
    }

    gatt_client_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);
    gap_set_connection_parameters(
        kScanInterval,
        kScanWindow,
        kFastConnIntervalMin,
        kFastConnIntervalMax,
        kFastConnLatency,
        kFastConnSupervisionTimeout,
        0,
        0
    );

    g_hci_callback_registration.callback = &ns2pro_ble_packet_handler;
    hci_add_event_handler(&g_hci_callback_registration);

    g_sm_callback_registration.callback = &ns2pro_ble_packet_handler;
    sm_add_event_handler(&g_sm_callback_registration);

    g_stack_services_registered = true;
    g_stack_ready = g_stack_ready || bt_stack_ready();
}

void ns2pro_ble_request_pairing() {
    ns2pro_ble_stack_init();
    if (!g_pairing_service_enabled) {
        pico_cmd_set_ns2pro_ble_state(g_ble_connected ? NS2PRO_BLE_STATE_READY : NS2PRO_BLE_STATE_IDLE);
        return;
    }
    if (bt_classic_connected()) {
        pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
        return;
    }
    bt_set_classic_pairing_service_enabled(true);
    load_saved_target_from_config();
    pico_cmd_set_ns2pro_ble_last_error(0);
    pico_cmd_clear_ns2pro_ble_debug();
    g_auto_reconnect_suppressed = false;
    g_auto_reconnect_resume_ms = 0;
    g_next_idle_task_time = nil_time;
    g_manual_pairing_deadline = delayed_by_ms(get_absolute_time(), kManualPairingWindowMs);

    if (g_ble_connected && g_input_stream_ready) {
        pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_READY);
        return;
    }
    if (g_ble_connected && g_conn_handle != HCI_CON_HANDLE_INVALID) {
        clear_characteristics();
        reset_latest_input();
        pico_cmd_set_ns2pro_connected(false);
        pico_cmd_set_ns2pro_battery_percent(0);
        gap_disconnect(g_conn_handle);
        return;
    }
    begin_scan_session(true, false, false);
}

static void maybe_begin_idle_pairing_session() {
    if (!g_runtime_initialized || !g_pairing_service_enabled || !g_idle_pairing_enabled) {
        return;
    }
    if (bt_classic_connected() || g_ble_connected || g_ble_connecting || g_pairing_requested) {
        return;
    }
    ns2pro_ble_stack_init();
    if (!g_stack_ready) {
        return;
    }
    bt_set_classic_pairing_service_enabled(true);
    pico_cmd_set_ns2pro_ble_last_error(0);
    pico_cmd_clear_ns2pro_ble_debug();
    g_auto_reconnect_suppressed = false;
    g_auto_reconnect_resume_ms = 0;
    g_next_idle_task_time = nil_time;
    begin_scan_session(true, true, false);
}

void ns2pro_ble_clear_bond() {
    stop_ble_scan();
    g_pairing_requested = false;
    g_pairing_prefer_new_target = false;
    g_alternating_scan = false;
    g_auto_reconnect_suppressed = false;
    g_auto_reconnect_resume_ms = 0;
    g_next_idle_task_time = nil_time;
    g_manual_pairing_deadline = nil_time;
    reset_home_disconnect_tracking();
    pico_cmd_clear_ns2pro_ble_debug();
    delete_saved_target_bonding();
    delete_active_target_bonding();
    g_saved_target_valid = false;
    g_active_target_valid = false;
    std::memset(g_saved_target_addr, 0, sizeof(g_saved_target_addr));
    std::memset(g_active_target_addr, 0, sizeof(g_active_target_addr));
    g_saved_target_addr_type = BD_ADDR_TYPE_LE_PUBLIC;
    g_active_target_addr_type = BD_ADDR_TYPE_LE_PUBLIC;
}

void ns2pro_ble_set_pairing_service_enabled(bool enabled) {
    g_pairing_service_enabled = enabled;
    if (!enabled) {
        stop_ble_scan();
        g_pairing_requested = false;
        g_pairing_prefer_new_target = false;
        g_alternating_scan = false;
        g_next_idle_task_time = nil_time;
        g_manual_pairing_deadline = nil_time;
        if (!g_ble_connected) {
            pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
        }
    }
}

bool ns2pro_ble_pairing_service_enabled() {
    return g_pairing_service_enabled;
}

void ns2pro_ble_set_idle_pairing_enabled(bool enabled) {
    g_idle_pairing_enabled = enabled;
    if (!enabled) {
        g_next_idle_task_time = nil_time;
    }
}

bool ns2pro_ble_idle_pairing_enabled() {
    return g_idle_pairing_enabled;
}

bool ns2pro_ble_poll_latest_input(uint8_t *payload, size_t *len) {
    if (!g_latest_input_ready || !payload || !len) {
        return false;
    }
    std::memcpy(payload, g_latest_input, g_latest_input_len);
    *len = g_latest_input_len;
    std::memcpy(g_last_dispatched_input, g_latest_input, g_latest_input_len);
    g_last_dispatched_input_len = g_latest_input_len;
    g_latest_input_ready = false;
    return true;
}

bool ns2pro_ble_connected() {
    return g_ble_connected;
}

bool ns2pro_ble_send_output_report(const uint8_t *report, size_t len) {
    uint8_t packet[kNs2ProBleRumblePacketLen];
    if (!build_rumble_packet_from_output_report(report, len, packet)) {
        g_last_output_status = 1;
        return false;
    }
    if (!g_ble_connected || g_conn_handle == HCI_CON_HANDLE_INVALID) {
        g_last_output_status = 2;
        return false;
    }
    if (!g_have_rumble || g_chr_rumble.value_handle == 0) {
        g_last_output_status = 3;
        return false;
    }

    if (g_rumble_write_without_response) {
        std::memcpy(g_pending_rumble_packet, packet, sizeof(packet));
        g_pending_rumble_valid = true;
        return request_pending_rumble_write();
    }

    if ((g_chr_rumble.properties & ATT_PROPERTY_WRITE) == 0) {
        g_last_output_status = 4;
        return false;
    }

    const uint8_t status = gatt_client_write_value_of_characteristic(
        &ns2pro_ble_packet_handler,
        g_conn_handle,
        g_chr_rumble.value_handle,
        sizeof(packet),
        packet
    );
    g_last_output_status = status == ERROR_CODE_SUCCESS ? 6 : static_cast<uint8_t>(0x40u | status);
    return status == ERROR_CODE_SUCCESS;
}

uint8_t ns2pro_ble_last_output_status() {
    return g_last_output_status;
}

void ns2pro_ble_on_classic_pair_success() {
    ns2pro_ble_set_pairing_service_enabled(false);
    bt_set_classic_pairing_service_enabled(false);
    stop_all_scans_for_success();
    pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
    g_next_idle_task_time = nil_time;
    g_manual_pairing_deadline = nil_time;
}

void ns2pro_ble_on_classic_disconnect() {
    if (!g_ble_connected && !g_ble_connecting) {
        ns2pro_ble_set_pairing_service_enabled(true);
        bt_set_classic_pairing_service_enabled(true);
        bt_set_classic_pairing_enabled(true);
    }
    g_next_idle_task_time = nil_time;
}

void ns2pro_ble_task() {
    if (!g_runtime_initialized || g_ble_connected) {
        return;
    }

    maybe_begin_idle_pairing_session();
    if (!g_stack_ready) {
        return;
    }

    const bool active_ble_work =
        g_pairing_requested || g_alternating_scan || g_ble_scan_active || g_ble_connecting;
    if (!active_ble_work) {
        const absolute_time_t now = get_absolute_time();
        if (!is_nil_time(g_next_idle_task_time) &&
            absolute_time_diff_us(now, g_next_idle_task_time) > 0) {
            return;
        }
        g_next_idle_task_time = delayed_by_ms(now, kIdleBleTaskIntervalMs);
    } else {
        g_next_idle_task_time = nil_time;
    }

    if (!g_pairing_service_enabled || !g_pairing_requested) {
        return;
    }

    if (!is_nil_time(g_manual_pairing_deadline) &&
        absolute_time_diff_us(get_absolute_time(), g_manual_pairing_deadline) <= 0) {
        stop_alternating_scan(true);
        pico_cmd_set_ns2pro_ble_state(NS2PRO_BLE_STATE_IDLE);
        g_manual_pairing_deadline = nil_time;
        return;
    }

    if (g_alternating_scan) {
        if (absolute_time_diff_us(g_last_slot_switch, get_absolute_time()) >=
            static_cast<int64_t>(kAlternateWindowMs) * 1000) {
            g_last_slot_switch = get_absolute_time();
            g_classic_scan_slot = !g_classic_scan_slot;
            if (g_classic_scan_slot) {
                stop_ble_scan();
                bt_set_classic_pairing_enabled(true);
            } else {
                bt_set_classic_pairing_enabled(false);
            }
        }
    }

    maybe_start_ble_scan();
}
