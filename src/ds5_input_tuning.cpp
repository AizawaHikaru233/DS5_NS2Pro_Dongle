#include "ds5_input_tuning.h"

#include <algorithm>
#include <cmath>

#include "config.h"

namespace {

struct DeadzoneLut {
    uint8_t deadzone_percent = 0xff;
    uint8_t values[256]{};
};

DeadzoneLut g_left_lut{};
DeadzoneLut g_right_lut{};

uint8_t compute_axis_deadzone(uint8_t value, uint8_t deadzone_percent) {
    const float deadzone = std::clamp(static_cast<float>(deadzone_percent) / 100.0f, 0.0f, 0.30f);
    if (deadzone <= 0.0f) {
        return value;
    }

    const float centered = (static_cast<float>(value) - 127.5f) / 127.5f;
    const float magnitude = std::fabs(centered);
    if (magnitude <= deadzone) {
        return 128;
    }

    const float scaled = (magnitude - deadzone) / std::max(0.001f, 1.0f - deadzone);
    const float tuned = std::clamp(scaled, 0.0f, 1.0f) * (centered < 0.0f ? -1.0f : 1.0f);
    const int encoded = static_cast<int>(std::lround((tuned * 127.5f) + 127.5f));
    return static_cast<uint8_t>(std::clamp(encoded, 0, 255));
}

void rebuild_deadzone_lut(DeadzoneLut *lut, uint8_t deadzone_percent) {
    if (!lut || lut->deadzone_percent == deadzone_percent) {
        return;
    }

    lut->deadzone_percent = deadzone_percent;
    for (int i = 0; i < 256; ++i) {
        lut->values[i] = compute_axis_deadzone(static_cast<uint8_t>(i), deadzone_percent);
    }
}

} // namespace

void ds5_apply_input_tuning(uint8_t *report63, size_t len) {
    if (!report63 || len < 4) {
        return;
    }

    const auto &cfg = get_config();
    if (cfg.ds5_left_stick_deadzone_percent == 0 &&
        cfg.ds5_right_stick_deadzone_percent == 0) {
        return;
    }

    rebuild_deadzone_lut(&g_left_lut, cfg.ds5_left_stick_deadzone_percent);
    rebuild_deadzone_lut(&g_right_lut, cfg.ds5_right_stick_deadzone_percent);

    report63[0] = g_left_lut.values[report63[0]];
    report63[1] = g_left_lut.values[report63[1]];
    report63[2] = g_right_lut.values[report63[2]];
    report63[3] = g_right_lut.values[report63[3]];
}
