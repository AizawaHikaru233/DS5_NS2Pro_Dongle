//
// Created by awalol on 2026/5/4.
//

#ifndef DS5_BRIDGE_CMD_H
#define DS5_BRIDGE_CMD_H

#include <stdint.h>

bool is_pico_cmd(uint8_t report_id);
uint16_t pico_cmd_get(uint8_t report_id, uint8_t *buffer,uint16_t reqlen);
void pico_cmd_set(uint8_t report_id, uint8_t const *buffer,uint16_t bufsize);
void pico_cmd_note_ds5_output_report(const uint8_t *report, uint8_t len);

enum : uint8_t {
    CMD_UPDATE_CONFIG = 0x01,
    CMD_SAVE_CONFIG = 0x02,
    CMD_RECONNECT_USB = 0x03,
    CMD_PREPARE_DUALSENSE_RUNTIME = 0x04,
    CMD_SET_INPUT_OWNER = 0x11,
    CMD_UPDATE_NS2PRO_CONFIG = 0x21,
    CMD_SAVE_NS2PRO_CONFIG = 0x22,
    CMD_NS2PRO_RUMBLE_SELF_TEST = 0x23,
    CMD_NS2PRO_RUMBLE_QUAD_SELF_TEST = 0x24,
    CMD_UPDATE_DS5_BUTTON_MAPPING = 0x50,
    CMD_SAVE_BUTTON_MAPPING = 0x51,
    CMD_UPDATE_NS2PRO_BUTTON_MAPPING = 0x52,
    CMD_NS2PRO_BLE_START_PAIRING = 0x40,
    CMD_NS2PRO_BLE_CLEAR_BOND = 0x41,
    CMD_NS2PRO_CALIBRATE_STICK_CENTER = 0x42,
    CMD_NS2PRO_CALIBRATE_GYRO_CENTER = 0x43,
};

enum : uint8_t {
    BRIDGE_BACKEND_BT_DS5 = 0,
    BRIDGE_BACKEND_NS2PRO = 1,
};

enum : uint8_t {
    BRIDGE_INPUT_OWNER_AUTO = 0,
    BRIDGE_INPUT_OWNER_DS5 = 1,
    BRIDGE_INPUT_OWNER_NS2PRO = 2,
};

enum : uint8_t {
    NS2PRO_BLE_STATE_DISABLED = 0,
    NS2PRO_BLE_STATE_IDLE = 1,
    NS2PRO_BLE_STATE_PAIRING_REQUESTED = 2,
    NS2PRO_BLE_STATE_SCANNING = 3,
    NS2PRO_BLE_STATE_CONNECTING = 4,
    NS2PRO_BLE_STATE_INITIALIZING = 5,
    NS2PRO_BLE_STATE_READY = 6,
    NS2PRO_BLE_STATE_ERROR = 7,
    NS2PRO_BLE_STATE_UNSUPPORTED = 8,
};

struct __attribute__((packed)) BridgeStatusPayload {
    uint8_t version;
    uint8_t backend;
    uint8_t reserved2;
    uint8_t last_error;
    uint8_t reserved4[20];
    uint8_t input_owner;
    uint8_t ds5_connected;
    uint8_t ns2pro_connected;
    uint8_t input_owner_policy;
    uint8_t ns2pro_battery_percent;
    int8_t signal_strength;
    uint8_t ns2pro_ble_state;
    uint8_t ns2pro_ble_last_error;
    uint8_t ns2pro_ble_has_bond;
    uint8_t reserved33[24];
    uint8_t rumble_sequence;
    uint8_t rumble_low;
    uint8_t rumble_high;
    uint8_t rumble_source;
    uint8_t ble_sent_count;
    uint8_t usb_queued_count;
};

struct Ns2ProRumbleDebug;

struct Ns2ProCommandDebug {
    uint8_t last_command;
    uint8_t self_test_count;
    uint8_t self_test_low;
    uint8_t self_test_high;
};

void pico_cmd_set_last_error(uint8_t error);
void pico_cmd_set_input_owner(uint8_t owner);
void pico_cmd_set_input_owner_policy(uint8_t owner);
void pico_cmd_set_ds5_connected(bool connected);
void pico_cmd_set_ns2pro_connected(bool connected);
void pico_cmd_set_ns2pro_battery_percent(uint8_t percent);
void pico_cmd_set_ns2pro_ble_state(uint8_t state);
void pico_cmd_set_ns2pro_ble_last_error(uint8_t error);
void pico_cmd_set_ns2pro_ble_has_bond(bool has_bond);
void pico_cmd_set_ns2pro_ble_pairing_debug(uint8_t status, uint8_t reason);
void pico_cmd_set_ns2pro_ble_reencryption_status(uint8_t status);
void pico_cmd_set_ns2pro_ble_disconnect_reason(uint8_t reason);
void pico_cmd_set_ns2pro_ble_gatt_debug(uint8_t att_status, uint8_t stage);
void pico_cmd_clear_ns2pro_ble_debug();
uint8_t pico_cmd_get_ns2pro_ble_state();
uint8_t pico_cmd_get_ns2pro_ble_last_error();
void pico_cmd_set_ns2pro_rumble_debug(const Ns2ProRumbleDebug &debug);
void pico_cmd_note_ns2pro_command(uint8_t command, uint8_t arg0, uint8_t arg1);

#endif //DS5_BRIDGE_CMD_H
