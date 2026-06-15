#ifndef DS5_BRIDGE_MANAGER_CONFIG_H
#define DS5_BRIDGE_MANAGER_CONFIG_H

#include <cstddef>
#include <cstdint>

struct __attribute__((packed)) BridgeManagerConfigBody {
    uint8_t config_version;
    float ds5_haptics_gain;
    float speaker_volume;
    uint8_t inactive_time;
    uint8_t disable_inactive_disconnect;
    uint8_t disable_pico_led;
    uint8_t polling_rate_mode;
    uint8_t haptics_buffer_length;
    uint8_t controller_mode;
    float ns2pro_rumble_gain;
    uint8_t ns2pro_rumble_style;
    uint8_t ns2pro_ble_has_target;
    uint8_t ns2pro_ble_address_type;
    uint8_t ns2pro_ble_address[6];
    uint8_t ds5_left_stick_deadzone_percent;
    uint8_t ds5_right_stick_deadzone_percent;
    uint8_t ns2pro_left_stick_deadzone_percent;
    uint8_t ns2pro_right_stick_deadzone_percent;
    uint8_t ns2pro_auto_stick_center;
};

static_assert(sizeof(BridgeManagerConfigBody) == 33);

size_t bridge_manager_config_get(uint8_t *buffer, uint16_t reqlen);
void bridge_manager_config_set(const uint8_t *buffer, uint16_t len);
bool bridge_manager_config_save();

#endif
