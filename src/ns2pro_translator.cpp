#include "ns2pro_translator.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "ns2pro_config.h"

namespace {

enum Ns2ProButtons : uint32_t {
    ButtonB = 1u << 0,
    ButtonA = 1u << 1,
    ButtonY = 1u << 2,
    ButtonX = 1u << 3,
    ButtonR = 1u << 4,
    ButtonZR = 1u << 5,
    ButtonPlus = 1u << 6,
    ButtonRightStick = 1u << 7,
    ButtonDown = 1u << 8,
    ButtonRight = 1u << 9,
    ButtonLeft = 1u << 10,
    ButtonUp = 1u << 11,
    ButtonL = 1u << 12,
    ButtonZL = 1u << 13,
    ButtonMinus = 1u << 14,
    ButtonLeftStick = 1u << 15,
    ButtonHome = 1u << 16,
    ButtonCapture = 1u << 17,
    ButtonGR = 1u << 18,
    ButtonGL = 1u << 19,
};

uint16_t unpack12_x(const uint8_t *data) {
    return static_cast<uint16_t>(data[0] | ((data[1] & 0x0f) << 8));
}

uint16_t unpack12_y(const uint8_t *data) {
    return static_cast<uint16_t>((data[1] >> 4) | (data[2] << 4));
}

uint32_t map_buttons(const uint8_t *raw) {
    uint32_t buttons = 0;
    if (raw[0] & 0x01) buttons |= ButtonY;
    if (raw[0] & 0x02) buttons |= ButtonX;
    if (raw[0] & 0x04) buttons |= ButtonB;
    if (raw[0] & 0x08) buttons |= ButtonA;
    if (raw[0] & 0x40) buttons |= ButtonR;
    if (raw[0] & 0x80) buttons |= ButtonZR;
    if (raw[1] & 0x01) buttons |= ButtonMinus;
    if (raw[1] & 0x02) buttons |= ButtonPlus;
    if (raw[1] & 0x04) buttons |= ButtonRightStick;
    if (raw[1] & 0x08) buttons |= ButtonLeftStick;
    if (raw[1] & 0x10) buttons |= ButtonHome;
    if (raw[1] & 0x20) buttons |= ButtonCapture;
    if (raw[2] & 0x01) buttons |= ButtonDown;
    if (raw[2] & 0x02) buttons |= ButtonUp;
    if (raw[2] & 0x04) buttons |= ButtonRight;
    if (raw[2] & 0x08) buttons |= ButtonLeft;
    if (raw[2] & 0x40) buttons |= ButtonL;
    if (raw[2] & 0x80) buttons |= ButtonZL;
    if (raw[3] & 0x01) buttons |= ButtonGR;
    if (raw[3] & 0x02) buttons |= ButtonGL;
    return buttons;
}

int16_t read_i16(const uint8_t *data, size_t offset) {
    return static_cast<int16_t>(data[offset] | (data[offset + 1] << 8));
}

uint8_t read_u8(const uint8_t *data, size_t offset, size_t len, uint8_t fallback) {
    return offset < len ? data[offset] : fallback;
}

struct StickVector {
    double x;
    double y;
};

constexpr int16_t kLeftCenterTrimX = -23;
constexpr int16_t kLeftCenterTrimY = 100;
constexpr int16_t kRightCenterTrimX = 105;
constexpr int16_t kRightCenterTrimY = 118;
Ns2ProInputState g_latest_input_state{};
bool g_have_latest_input_state = false;

StickVector normalize_stick_pair(
    uint16_t raw_x,
    uint16_t raw_y,
    int16_t center_x,
    int16_t center_y,
    uint8_t deadzone_percent,
    double physical_range,
    double leftward_boost,
    double rightward_boost,
    double upward_boost,
    double downward_boost,
    double overflow_gain
) {
    constexpr double kCenter = 2048.0;
    constexpr double kRawSpan = 2047.0;

    const double center_dx = static_cast<double>(raw_x) - static_cast<double>(center_x);
    const double center_dy = static_cast<double>(raw_y) - static_cast<double>(center_y);

    double x = center_dx / kRawSpan;
    double y = -(center_dy / kRawSpan);

    if (x < 0.0) {
        x *= leftward_boost;
    } else if (x > 0.0) {
        x *= rightward_boost;
    }
    if (y < 0.0) {
        y *= downward_boost;
    } else if (y > 0.0) {
        y *= upward_boost;
    }

    const double magnitude = std::sqrt((x * x) + (y * y));
    const double deadzone = std::clamp(static_cast<double>(deadzone_percent) / 100.0, 0.0, 0.30);
    const double full_scale = std::clamp(physical_range / kRawSpan, 0.1, 1.0);
    if (magnitude < 1e-9 || magnitude <= deadzone) {
        return {0.0, 0.0};
    }

    const double unit_x = x / magnitude;
    const double unit_y = y / magnitude;
    double scaled_magnitude = (magnitude - deadzone) / std::max(1e-6, full_scale - deadzone);
    scaled_magnitude = std::clamp(scaled_magnitude * overflow_gain, 0.0, 1.08);
    return {unit_x * scaled_magnitude, unit_y * scaled_magnitude};
}

uint8_t stick_component_to_ds5(double component) {
    const double clamped = std::clamp(component, -1.0, 1.0);
    const int scaled = static_cast<int>(std::lround(((clamped + 1.0) * 0.5) * 255.0));
    return static_cast<uint8_t>(std::clamp(scaled, 0, 255));
}

uint8_t encode_hat(uint32_t buttons) {
    const bool up = (buttons & ButtonUp) != 0;
    const bool down = (buttons & ButtonDown) != 0;
    const bool left = (buttons & ButtonLeft) != 0;
    const bool right = (buttons & ButtonRight) != 0;

    if (up && right) return 1;
    if (right && down) return 3;
    if (down && left) return 5;
    if (left && up) return 7;
    if (up) return 0;
    if (right) return 2;
    if (down) return 4;
    if (left) return 6;
    return 8;
}

void write_i16(uint8_t *data, size_t offset, int16_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xff);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

void write_u32(uint8_t *data, size_t offset, uint32_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xff);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
    data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xff);
    data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xff);
}

uint8_t encode_battery(const Ns2ProInputState &state) {
    uint8_t level = static_cast<uint8_t>(std::min<int>(10, (state.battery_percent + 9) / 10));
    uint8_t charge = 0x00;
    if (state.external_power && state.charging) {
        charge = 0x10;
    } else if (state.external_power) {
        charge = 0x20;
        level = 0x0a;
    }
    return static_cast<uint8_t>(charge | (level & 0x0f));
}

} // namespace

bool ns2pro_parse_input_report(const uint8_t *payload, size_t len, Ns2ProInputState *out_state) {
    if (!payload || !out_state || len < 0x3c) {
        return false;
    }

    const auto &cfg = get_ns2pro_config();
    Ns2ProInputState state{};
    state.buttons = map_buttons(payload + 0x04);
    state.lx = unpack12_x(payload + 0x0a);
    state.ly = unpack12_y(payload + 0x0a);
    state.rx = unpack12_x(payload + 0x0d);
    state.ry = unpack12_y(payload + 0x0d);
    state.accel_x = read_i16(payload, 0x30);
    state.accel_y = read_i16(payload, 0x32);
    state.accel_z = read_i16(payload, 0x34);
    // NS2Pro raw payload stores gyro channels at 0x36/0x38/0x3a.
    // Keep the logical state ordered as X/Y/Z here and leave any game-side
    // speed tuning to the DS5 0x05 calibration feature report layer.
    state.gyro_x = read_i16(payload, 0x36);
    state.gyro_z = read_i16(payload, 0x38);
    state.gyro_y = read_i16(payload, 0x3a);
    const uint16_t voltage = static_cast<uint16_t>(read_u8(payload, 0x1f, len, 0) | (read_u8(payload, 0x20, len, 0) << 8));
    state.battery_percent = voltage > 0 ? static_cast<uint8_t>(std::clamp(((static_cast<int>(voltage) - 3200) * 100) / 800, 0, 100)) : 100;
    state.external_power = true;
    state.charging = read_u8(payload, 0x21, len, 0) == 0x34;
    state.connected = true;

    if (cfg.auto_stick_center) {
        if (cfg.center_lx >= 0) state.lx = static_cast<uint16_t>(state.lx);
        if (cfg.center_ly >= 0) state.ly = static_cast<uint16_t>(state.ly);
        if (cfg.center_rx >= 0) state.rx = static_cast<uint16_t>(state.rx);
        if (cfg.center_ry >= 0) state.ry = static_cast<uint16_t>(state.ry);
    }

    *out_state = state;
    g_latest_input_state = state;
    g_have_latest_input_state = true;
    return true;
}

void ns2pro_build_ds5_input_report(const Ns2ProInputState &state, uint8_t *report63, uint32_t tick_counter) {
    static const uint8_t kDefaultReport[63] = {
        0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
        0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
        0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
        0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
        0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
        0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
        0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
    };

    memcpy(report63, kDefaultReport, sizeof(kDefaultReport));
    const auto &cfg = get_ns2pro_config();
    const int16_t left_center_x = static_cast<int16_t>(std::clamp<int>(cfg.center_lx + kLeftCenterTrimX, 0, 4095));
    const int16_t left_center_y = static_cast<int16_t>(std::clamp<int>(cfg.center_ly + kLeftCenterTrimY, 0, 4095));
    const int16_t right_center_x = static_cast<int16_t>(std::clamp<int>(cfg.center_rx + kRightCenterTrimX, 0, 4095));
    const int16_t right_center_y = static_cast<int16_t>(std::clamp<int>(cfg.center_ry + kRightCenterTrimY, 0, 4095));

    const StickVector left_stick = normalize_stick_pair(
        state.lx,
        state.ly,
        left_center_x,
        left_center_y,
        cfg.left_stick_deadzone_percent,
        1550.0,
        1.0,
        1.03,
        1.0,
        1.12,
        1.04
    );
    const StickVector right_stick = normalize_stick_pair(
        state.rx,
        state.ry,
        right_center_x,
        right_center_y,
        cfg.right_stick_deadzone_percent,
        1570.0,
        1.06,
        1.0,
        1.0,
        1.05,
        1.04
    );

    report63[0] = stick_component_to_ds5(left_stick.x);
    report63[1] = stick_component_to_ds5(left_stick.y);
    report63[2] = stick_component_to_ds5(right_stick.x);
    report63[3] = stick_component_to_ds5(right_stick.y);
    report63[4] = (state.buttons & ButtonZL) ? 0xff : 0x00;
    report63[5] = (state.buttons & ButtonZR) ? 0xff : 0x00;
    report63[6] = static_cast<uint8_t>(tick_counter & 0xff);

    uint8_t buttons0 = encode_hat(state.buttons);
    if (state.buttons & ButtonY) buttons0 |= 1u << 4; // Square
    if (state.buttons & ButtonB) buttons0 |= 1u << 5; // Cross
    if (state.buttons & ButtonA) buttons0 |= 1u << 6; // Circle
    if (state.buttons & ButtonX) buttons0 |= 1u << 7; // Triangle
    report63[7] = buttons0;

    uint8_t buttons1 = 0;
    if (state.buttons & ButtonL) buttons1 |= 1u << 0;
    if (state.buttons & ButtonR) buttons1 |= 1u << 1;
    if (state.buttons & ButtonZL) buttons1 |= 1u << 2;
    if (state.buttons & ButtonZR) buttons1 |= 1u << 3;
    if (state.buttons & ButtonMinus) buttons1 |= 1u << 4;
    if (state.buttons & ButtonPlus) buttons1 |= 1u << 5;
    if (state.buttons & ButtonLeftStick) buttons1 |= 1u << 6;
    if (state.buttons & ButtonRightStick) buttons1 |= 1u << 7;
    report63[8] = buttons1;

    uint8_t buttons2 = 0;
    if (state.buttons & ButtonHome) buttons2 |= 1u << 0;
    if (state.buttons & ButtonCapture) buttons2 |= 1u << 1;
    if (state.buttons & ButtonGL) buttons2 |= 1u << 6;
    if (state.buttons & ButtonGR) buttons2 |= 1u << 7;
    report63[9] = buttons2;

    write_i16(report63, 15, state.gyro_x);
    const int16_t ds5_gyro_y = cfg.gyro_invert_y ? state.gyro_y : static_cast<int16_t>(-state.gyro_y);
    write_i16(report63, 17, ds5_gyro_y);
    write_i16(report63, 19, static_cast<int16_t>(-state.gyro_z));
    write_i16(report63, 21, state.accel_x);
    write_i16(report63, 23, state.accel_y);
    write_i16(report63, 25, static_cast<int16_t>(-state.accel_z));
    write_u32(report63, 27, tick_counter * 188u);

    report63[52] = encode_battery(state);
    report63[53] = state.connected ? 0x08 : 0x00;
}

bool ns2pro_calibrate_stick_center_from_latest() {
    if (!g_have_latest_input_state) {
        return false;
    }

    auto cfg = get_ns2pro_config();
    cfg.center_lx = static_cast<int16_t>(std::clamp<int>(g_latest_input_state.lx, 0, 4095));
    cfg.center_ly = static_cast<int16_t>(std::clamp<int>(g_latest_input_state.ly, 0, 4095));
    cfg.center_rx = static_cast<int16_t>(std::clamp<int>(g_latest_input_state.rx, 0, 4095));
    cfg.center_ry = static_cast<int16_t>(std::clamp<int>(g_latest_input_state.ry, 0, 4095));
    set_ns2pro_config(cfg);
    return true;
}
