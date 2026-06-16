#ifndef DS5_BRIDGE_NS2PRO_CONFIG_H
#define DS5_BRIDGE_NS2PRO_CONFIG_H

#include <cstdint>

struct __attribute__((packed)) Ns2ProConfigBody {
    uint8_t config_version;
    float rumble_gain;
    uint8_t rumble_style;
    uint8_t ble_has_target;
    uint8_t ble_address_type;
    uint8_t ble_address[6];
    uint8_t left_stick_deadzone_percent;
    uint8_t right_stick_deadzone_percent;
    uint8_t auto_stick_center;
    int16_t center_lx;
    int16_t center_ly;
    int16_t center_rx;
    int16_t center_ry;
    uint8_t gyro_invert_y;
    uint8_t auto_gyro_center;
    int16_t gyro_center_x;
    int16_t gyro_center_y;
    int16_t gyro_center_z;
};

struct __attribute__((packed)) Ns2ProConfig {
    uint32_t magic;
    uint32_t crc32;
    uint16_t size;
    Ns2ProConfigBody body;
};

void ns2pro_config_default();
void ns2pro_config_load();
bool ns2pro_config_save();
void ns2pro_config_valid();
Ns2ProConfigBody &get_ns2pro_config();
void set_ns2pro_config(const uint8_t *new_config, uint16_t len);
void set_ns2pro_config(const Ns2ProConfigBody &new_config);

#endif
