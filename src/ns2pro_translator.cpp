#include "ns2pro_translator.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "button_mapping.h"
#include "ns2pro_config.h"

namespace {

uint16_t unpack12_x(const uint8_t *data) {
    return static_cast<uint16_t>(data[0] | ((data[1] & 0x0f) << 8));
}

uint16_t unpack12_y(const uint8_t *data) {
    return static_cast<uint16_t>((data[1] >> 4) | (data[2] << 4));
}

uint32_t map_buttons(const uint8_t *raw) {
    uint32_t buttons = 0;
    if (raw[0] & 0x01) buttons |= 1u << kNs2ProButtonInputY;
    if (raw[0] & 0x02) buttons |= 1u << kNs2ProButtonInputX;
    if (raw[0] & 0x04) buttons |= 1u << kNs2ProButtonInputB;
    if (raw[0] & 0x08) buttons |= 1u << kNs2ProButtonInputA;
    if (raw[0] & 0x40) buttons |= 1u << kNs2ProButtonInputR;
    if (raw[0] & 0x80) buttons |= 1u << kNs2ProButtonInputZR;
    if (raw[1] & 0x01) buttons |= 1u << kNs2ProButtonInputMinus;
    if (raw[1] & 0x02) buttons |= 1u << kNs2ProButtonInputPlus;
    if (raw[1] & 0x04) buttons |= 1u << kNs2ProButtonInputR3;
    if (raw[1] & 0x08) buttons |= 1u << kNs2ProButtonInputL3;
    if (raw[1] & 0x10) buttons |= 1u << kNs2ProButtonInputHome;
    if (raw[1] & 0x20) buttons |= 1u << kNs2ProButtonInputCapture;
    if (raw[2] & 0x01) buttons |= 1u << kNs2ProButtonInputDown;
    if (raw[2] & 0x02) buttons |= 1u << kNs2ProButtonInputUp;
    if (raw[2] & 0x04) buttons |= 1u << kNs2ProButtonInputRight;
    if (raw[2] & 0x08) buttons |= 1u << kNs2ProButtonInputLeft;
    if (raw[2] & 0x40) buttons |= 1u << kNs2ProButtonInputL;
    if (raw[2] & 0x80) buttons |= 1u << kNs2ProButtonInputZL;
    if (raw[3] & 0x01) buttons |= 1u << kNs2ProButtonInputGR;
    if (raw[3] & 0x02) buttons |= 1u << kNs2ProButtonInputGL;
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
constexpr uint8_t kIdleCalibrationFrames = 32;
constexpr int kIdleStickThreshold = 120;
constexpr int16_t kIdleGyroThreshold = 800;
Ns2ProInputState g_latest_input_state{};
bool g_have_latest_input_state = false;
bool g_runtime_calibration_initialized = false;
int16_t g_runtime_center_lx = 2048;
int16_t g_runtime_center_ly = 2048;
int16_t g_runtime_center_rx = 2048;
int16_t g_runtime_center_ry = 2048;
int16_t g_runtime_gyro_center_x = 0;
int16_t g_runtime_gyro_center_y = 0;
int16_t g_runtime_gyro_center_z = 0;
uint8_t g_idle_calibration_frames = 0;
int32_t g_idle_sum_lx = 0;
int32_t g_idle_sum_ly = 0;
int32_t g_idle_sum_rx = 0;
int32_t g_idle_sum_ry = 0;
int32_t g_idle_sum_gyro_x = 0;
int32_t g_idle_sum_gyro_y = 0;
int32_t g_idle_sum_gyro_z = 0;

int16_t clamp_center_value(int value) {
    return static_cast<int16_t>(std::clamp(value, 0, 4095));
}

int16_t runtime_center_to_effective_center(int16_t runtime_center, int16_t trim) {
    return clamp_center_value(static_cast<int>(runtime_center) + static_cast<int>(trim));
}

int16_t raw_center_to_runtime_center(int16_t raw_center, int16_t trim) {
    return clamp_center_value(static_cast<int>(raw_center) - static_cast<int>(trim));
}

void reset_idle_calibration_samples() {
    g_idle_calibration_frames = 0;
    g_idle_sum_lx = 0;
    g_idle_sum_ly = 0;
    g_idle_sum_rx = 0;
    g_idle_sum_ry = 0;
    g_idle_sum_gyro_x = 0;
    g_idle_sum_gyro_y = 0;
    g_idle_sum_gyro_z = 0;
}

void ensure_runtime_calibration_initialized() {
    if (g_runtime_calibration_initialized) {
        return;
    }

    const auto &cfg = get_ns2pro_config();
    g_runtime_center_lx = cfg.center_lx;
    g_runtime_center_ly = cfg.center_ly;
    g_runtime_center_rx = cfg.center_rx;
    g_runtime_center_ry = cfg.center_ry;
    g_runtime_gyro_center_x = cfg.gyro_center_x;
    g_runtime_gyro_center_y = cfg.gyro_center_y;
    g_runtime_gyro_center_z = cfg.gyro_center_z;
    g_runtime_calibration_initialized = true;
    reset_idle_calibration_samples();
}

bool is_idle_candidate(const Ns2ProInputState &state) {
    if (state.buttons != 0) {
        return false;
    }

    const int16_t left_center_x = runtime_center_to_effective_center(g_runtime_center_lx, kLeftCenterTrimX);
    const int16_t left_center_y = runtime_center_to_effective_center(g_runtime_center_ly, kLeftCenterTrimY);
    const int16_t right_center_x = runtime_center_to_effective_center(g_runtime_center_rx, kRightCenterTrimX);
    const int16_t right_center_y = runtime_center_to_effective_center(g_runtime_center_ry, kRightCenterTrimY);

    const bool sticks_centered =
        std::abs(static_cast<int>(state.lx) - left_center_x) <= kIdleStickThreshold &&
        std::abs(static_cast<int>(state.ly) - left_center_y) <= kIdleStickThreshold &&
        std::abs(static_cast<int>(state.rx) - right_center_x) <= kIdleStickThreshold &&
        std::abs(static_cast<int>(state.ry) - right_center_y) <= kIdleStickThreshold;
    if (!sticks_centered) {
        return false;
    }

    return
        std::abs(state.gyro_x - g_runtime_gyro_center_x) <= kIdleGyroThreshold &&
        std::abs(state.gyro_y - g_runtime_gyro_center_y) <= kIdleGyroThreshold &&
        std::abs(state.gyro_z - g_runtime_gyro_center_z) <= kIdleGyroThreshold;
}

void update_idle_auto_calibration(const Ns2ProInputState &raw_state) {
    ensure_runtime_calibration_initialized();
    const auto &cfg = get_ns2pro_config();
    if (!cfg.auto_stick_center && !cfg.auto_gyro_center) {
        reset_idle_calibration_samples();
        return;
    }

    if (!is_idle_candidate(raw_state)) {
        reset_idle_calibration_samples();
        return;
    }

    g_idle_sum_lx += raw_state.lx;
    g_idle_sum_ly += raw_state.ly;
    g_idle_sum_rx += raw_state.rx;
    g_idle_sum_ry += raw_state.ry;
    g_idle_sum_gyro_x += raw_state.gyro_x;
    g_idle_sum_gyro_y += raw_state.gyro_y;
    g_idle_sum_gyro_z += raw_state.gyro_z;
    g_idle_calibration_frames += 1;

    if (g_idle_calibration_frames < kIdleCalibrationFrames) {
        return;
    }

    if (cfg.auto_stick_center) {
        g_runtime_center_lx = raw_center_to_runtime_center(
            static_cast<int16_t>(g_idle_sum_lx / kIdleCalibrationFrames),
            kLeftCenterTrimX
        );
        g_runtime_center_ly = raw_center_to_runtime_center(
            static_cast<int16_t>(g_idle_sum_ly / kIdleCalibrationFrames),
            kLeftCenterTrimY
        );
        g_runtime_center_rx = raw_center_to_runtime_center(
            static_cast<int16_t>(g_idle_sum_rx / kIdleCalibrationFrames),
            kRightCenterTrimX
        );
        g_runtime_center_ry = raw_center_to_runtime_center(
            static_cast<int16_t>(g_idle_sum_ry / kIdleCalibrationFrames),
            kRightCenterTrimY
        );
    }
    if (cfg.auto_gyro_center) {
        g_runtime_gyro_center_x = static_cast<int16_t>(g_idle_sum_gyro_x / kIdleCalibrationFrames);
        g_runtime_gyro_center_y = static_cast<int16_t>(g_idle_sum_gyro_y / kIdleCalibrationFrames);
        g_runtime_gyro_center_z = static_cast<int16_t>(g_idle_sum_gyro_z / kIdleCalibrationFrames);
    }
    reset_idle_calibration_samples();
}

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
    const bool up = (buttons & (1u << kButtonMappingTargetUp)) != 0;
    const bool down = (buttons & (1u << kButtonMappingTargetDown)) != 0;
    const bool left = (buttons & (1u << kButtonMappingTargetLeft)) != 0;
    const bool right = (buttons & (1u << kButtonMappingTargetRight)) != 0;

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

    ensure_runtime_calibration_initialized();
    Ns2ProInputState state{};
    const uint32_t physical_buttons = map_buttons(payload + 0x04);
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
    update_idle_auto_calibration(state);
    state.buttons = button_mapping_apply_ns2pro(physical_buttons);
    state.gyro_x = static_cast<int16_t>(state.gyro_x - g_runtime_gyro_center_x);
    state.gyro_y = static_cast<int16_t>(state.gyro_y - g_runtime_gyro_center_y);
    state.gyro_z = static_cast<int16_t>(state.gyro_z - g_runtime_gyro_center_z);

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
    ensure_runtime_calibration_initialized();
    const int16_t left_center_x = runtime_center_to_effective_center(g_runtime_center_lx, kLeftCenterTrimX);
    const int16_t left_center_y = runtime_center_to_effective_center(g_runtime_center_ly, kLeftCenterTrimY);
    const int16_t right_center_x = runtime_center_to_effective_center(g_runtime_center_rx, kRightCenterTrimX);
    const int16_t right_center_y = runtime_center_to_effective_center(g_runtime_center_ry, kRightCenterTrimY);

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
    report63[4] = (state.buttons & (1u << kButtonMappingTargetL2)) ? 0xff : 0x00;
    report63[5] = (state.buttons & (1u << kButtonMappingTargetR2)) ? 0xff : 0x00;
    report63[6] = static_cast<uint8_t>(tick_counter & 0xff);

    uint8_t buttons0 = encode_hat(state.buttons);
    if (state.buttons & (1u << kButtonMappingTargetSquare)) buttons0 |= 1u << 4;
    if (state.buttons & (1u << kButtonMappingTargetCross)) buttons0 |= 1u << 5;
    if (state.buttons & (1u << kButtonMappingTargetCircle)) buttons0 |= 1u << 6;
    if (state.buttons & (1u << kButtonMappingTargetTriangle)) buttons0 |= 1u << 7;
    report63[7] = buttons0;

    uint8_t buttons1 = 0;
    if (state.buttons & (1u << kButtonMappingTargetL1)) buttons1 |= 1u << 0;
    if (state.buttons & (1u << kButtonMappingTargetR1)) buttons1 |= 1u << 1;
    if (state.buttons & (1u << kButtonMappingTargetL2)) buttons1 |= 1u << 2;
    if (state.buttons & (1u << kButtonMappingTargetR2)) buttons1 |= 1u << 3;
    if (state.buttons & (1u << kButtonMappingTargetCreate)) buttons1 |= 1u << 4;
    if (state.buttons & (1u << kButtonMappingTargetOptions)) buttons1 |= 1u << 5;
    if (state.buttons & (1u << kButtonMappingTargetL3)) buttons1 |= 1u << 6;
    if (state.buttons & (1u << kButtonMappingTargetR3)) buttons1 |= 1u << 7;
    report63[8] = buttons1;

    uint8_t buttons2 = 0;
    if (state.buttons & (1u << kButtonMappingTargetPs)) buttons2 |= 1u << 0;
    if (state.buttons & (1u << kButtonMappingTargetTouchpad)) buttons2 |= 1u << 1;
    if (state.buttons & (1u << kButtonMappingTargetMute)) buttons2 |= 1u << 2;
    if (state.buttons & (1u << kButtonMappingTargetLeftFunction)) buttons2 |= 1u << 4;
    if (state.buttons & (1u << kButtonMappingTargetRightFunction)) buttons2 |= 1u << 5;
    if (state.buttons & (1u << kButtonMappingTargetLeftPaddle)) buttons2 |= 1u << 6;
    if (state.buttons & (1u << kButtonMappingTargetRightPaddle)) buttons2 |= 1u << 7;
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
    cfg.center_lx = raw_center_to_runtime_center(
        static_cast<int16_t>(std::clamp<int>(g_latest_input_state.lx, 0, 4095)),
        kLeftCenterTrimX
    );
    cfg.center_ly = raw_center_to_runtime_center(
        static_cast<int16_t>(std::clamp<int>(g_latest_input_state.ly, 0, 4095)),
        kLeftCenterTrimY
    );
    cfg.center_rx = raw_center_to_runtime_center(
        static_cast<int16_t>(std::clamp<int>(g_latest_input_state.rx, 0, 4095)),
        kRightCenterTrimX
    );
    cfg.center_ry = raw_center_to_runtime_center(
        static_cast<int16_t>(std::clamp<int>(g_latest_input_state.ry, 0, 4095)),
        kRightCenterTrimY
    );
    set_ns2pro_config(cfg);
    return true;
}

void ns2pro_reset_runtime_calibration_from_config() {
    const auto &cfg = get_ns2pro_config();
    g_runtime_center_lx = cfg.center_lx;
    g_runtime_center_ly = cfg.center_ly;
    g_runtime_center_rx = cfg.center_rx;
    g_runtime_center_ry = cfg.center_ry;
    g_runtime_gyro_center_x = cfg.gyro_center_x;
    g_runtime_gyro_center_y = cfg.gyro_center_y;
    g_runtime_gyro_center_z = cfg.gyro_center_z;
    g_runtime_calibration_initialized = true;
    reset_idle_calibration_samples();
}
