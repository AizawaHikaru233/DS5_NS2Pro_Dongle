#ifndef DS5_BRIDGE_NS2PRO_TRANSLATOR_H
#define DS5_BRIDGE_NS2PRO_TRANSLATOR_H

#include <cstddef>
#include <cstdint>

struct Ns2ProInputState {
    uint32_t buttons;
    uint16_t lx;
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t battery_percent;
    bool connected;
    bool external_power;
    bool charging;
};

bool ns2pro_parse_input_report(const uint8_t *payload, size_t len, Ns2ProInputState *out_state);
void ns2pro_build_ds5_input_report(const Ns2ProInputState &state, uint8_t *report63, uint32_t tick_counter);
bool ns2pro_calibrate_stick_center_from_latest();
void ns2pro_reset_runtime_calibration_from_config();

#endif
