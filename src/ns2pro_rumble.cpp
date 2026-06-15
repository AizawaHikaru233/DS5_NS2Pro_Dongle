#include "ns2pro_rumble.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "pico/time.h"
#include "ns2pro_config.h"
#include "utils.h"

namespace {

constexpr uint8_t kSourceNone = 0;
constexpr uint8_t kSourceRumble = 1;
constexpr uint8_t kSourceHaptics = 2;
constexpr uint8_t kSourceMixed = 3;
constexpr uint8_t kRumbleStyleDirect = 0;
constexpr uint8_t kRumbleStyleAudioHaptics = 1;

constexpr size_t kNs2ProOutputReportLen = 64;
constexpr uint16_t kHighFrequencyMin = 0x172;
constexpr uint16_t kHighFrequencyMax = 0x1A8;
constexpr uint16_t kLowFrequencyMin = 0x108;
constexpr uint16_t kLowFrequencyMax = 0x13E;
constexpr float kOutputStrengthScale = 1.44f;
constexpr float kAudioHapticsBaseStrengthScale = 0.5f;
constexpr float kHapticsSampleRateHz = 3000.0f;
constexpr float kHapticsCrossoverHz = 320.0f;
constexpr float kEasyInputNoiseGate = 0.003f;
constexpr float kEasyInputLowGain = 1.45f;
constexpr float kEasyInputHighGain = 1.45f;
constexpr float kEasyInputLowResponseCurve = 1.85f;
constexpr float kEasyInputHighResponseCurve = 1.85f;
constexpr bool kDisableLowMotorForTest = false;
constexpr bool kDisableHighMotorForTest = false;

struct RumbleState {
    uint8_t regular_right = 0;
    uint8_t regular_left = 0;
    uint8_t haptic_right_low = 0;
    uint8_t haptic_right_high = 0;
    uint8_t haptic_left_low = 0;
    uint8_t haptic_left_high = 0;
    uint8_t mixed_right_low = 0;
    uint8_t mixed_right_high = 0;
    uint8_t mixed_left_low = 0;
    uint8_t mixed_left_high = 0;
    uint8_t source = kSourceNone;
    uint8_t profile = kRumbleStyleDirect;
    uint8_t sequence = 0;
    uint8_t input_count = 0;
    uint8_t build_count = 0;
    uint8_t send_attempt_count = 0;
    uint8_t send_success_count = 0;
    uint8_t send_failure_count = 0;
    uint8_t last_send_stage = 0;
    uint8_t ble_sent_count = 0;
    uint8_t usb_queued_count = 0;
    uint8_t last_report_head[12] = {0};
    bool dirty = false;
    absolute_time_t next_stream_at{};
};

RumbleState g_rumble{};
float g_haptics_left_low_state = 0.0f;
float g_haptics_right_low_state = 0.0f;

uint8_t clamp_u8(int value) {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

uint8_t mix_peak(uint8_t a, uint8_t b) {
    return a > b ? a : b;
}

uint16_t clamp_frequency(int value) {
    return static_cast<uint16_t>(std::clamp(value, 0, 0x3FF));
}

float current_rumble_gain() {
    return std::clamp(get_ns2pro_config().rumble_gain, 0.0f, 2.0f);
}

uint16_t map_frequency(uint8_t strength, uint16_t min, uint16_t max, bool audio_haptics_style, bool high_band) {
    if (strength == 0) {
        return static_cast<uint16_t>((min + max) / 2);
    }

    const float t = static_cast<float>(strength) / 255.0f;
    if (audio_haptics_style) {
        const uint16_t lifted_min = high_band ? 0x184 : 0x118;
        const uint16_t lifted_max = high_band ? 0x1B0 : 0x148;
        const float shaped_audio = std::pow(t, 0.52f);
        const float lifted_value =
            static_cast<float>(lifted_min) +
            (static_cast<float>(lifted_max - lifted_min) * shaped_audio);
        return clamp_frequency(static_cast<int>(std::lround(lifted_value)));
    }

    const float value = static_cast<float>(min) + (static_cast<float>(max - min) * t);
    return clamp_frequency(static_cast<int>(std::lround(value)));
}

uint16_t map_amplitude(uint8_t strength, float scale = 1.0f) {
    const float amplitude = static_cast<float>(strength) * kOutputStrengthScale * current_rumble_gain() * scale;
    return static_cast<uint16_t>(std::clamp(static_cast<int>(std::lround(amplitude)), 0, 0x3FF));
}

float lowpass_alpha(float cutoff_hz, float sample_rate_hz) {
    const float dt = 1.0f / std::max(1.0f, sample_rate_hz);
    const float rc = 1.0f / (2.0f * 3.14159265358979323846f * std::max(1.0f, cutoff_hz));
    return dt / (rc + dt);
}

uint8_t map_easy_input_level(float input_level, float gain, float response_curve, float noise_gate) {
    const float normalized = std::clamp(input_level, 0.0f, 1.0f);
    if (normalized < noise_gate) {
        return 0;
    }

    const float effective = std::clamp(
        (normalized - noise_gate) / std::max(0.0001f, 1.0f - noise_gate),
        0.0f,
        1.0f
    );
    const float lifted = std::pow(effective, 1.0f / std::max(0.1f, response_curve));
    float curved = 1.0f - std::exp(-std::max(0.0f, gain) * lifted);
    if (effective > 0.0f) {
        curved = std::max(curved, std::min(0.035f, 0.012f + effective * 0.35f));
    }
    return clamp_u8(static_cast<int>(std::lround(std::clamp(curved, 0.0f, 1.0f) * 255.0f)));
}

void pack_rumble_frame(uint16_t hi_amp, uint16_t lo_amp, uint16_t hi_freq, uint16_t lo_freq, uint8_t out5[5]) {
    uint64_t packed = 0;
    packed |= static_cast<uint64_t>(hi_amp & 0x3FFu) << 30u;
    packed |= static_cast<uint64_t>(hi_freq & 0x3FFu) << 20u;
    packed |= static_cast<uint64_t>(lo_amp & 0x3FFu) << 10u;
    packed |= static_cast<uint64_t>(lo_freq & 0x3FFu);
    for (size_t i = 0; i < 5; ++i) {
        out5[i] = static_cast<uint8_t>((packed >> (i * 8u)) & 0xFFu);
    }
}

void build_rumble_frames(uint8_t strong, uint8_t weak, bool audio_haptics_style, uint8_t out[3][5]) {
    struct Profile {
        float hi_scale;
        float lo_scale;
        int hi_delta;
        int lo_delta;
    };
    static constexpr Profile kStandardProfiles[3] = {
        {1.04f, 0.98f, +0x07, +0x04},
        {0.72f, 0.68f, +0x00, -0x01},
        {0.42f, 0.38f, -0x06, -0x05},
    };
    static constexpr Profile kAudioProfiles[3] = {
        {1.08f, 0.96f, +0x0C, +0x05},
        {0.64f, 0.58f, +0x02, -0x01},
        {0.32f, 0.28f, -0x09, -0x07},
    };

    const uint16_t hi_freq = map_frequency(weak, kHighFrequencyMin, kHighFrequencyMax, audio_haptics_style, true);
    const uint16_t lo_freq = map_frequency(strong, kLowFrequencyMin, kLowFrequencyMax, audio_haptics_style, false);

    if (!audio_haptics_style) {
        const uint16_t hi_amp = kDisableHighMotorForTest ? 0 : map_amplitude(weak, 1.0f);
        const uint16_t lo_amp = kDisableLowMotorForTest ? 0 : map_amplitude(strong, 1.0f);
        for (size_t i = 0; i < 3; ++i) {
            pack_rumble_frame(hi_amp, lo_amp, hi_freq, lo_freq, out[i]);
        }
        return;
    }

    const Profile *profiles = kAudioProfiles;

    for (size_t i = 0; i < 3; ++i) {
        const auto &entry = profiles[i];
        const uint16_t hi_amp = kDisableHighMotorForTest ? 0 : map_amplitude(
            weak,
            entry.hi_scale * kAudioHapticsBaseStrengthScale
        );
        const uint16_t lo_amp = kDisableLowMotorForTest ? 0 : map_amplitude(
            strong,
            entry.lo_scale * kAudioHapticsBaseStrengthScale
        );
        pack_rumble_frame(
            hi_amp,
            lo_amp,
            clamp_frequency(static_cast<int>(hi_freq) + entry.hi_delta),
            clamp_frequency(static_cast<int>(lo_freq) + entry.lo_delta),
            out[i]
        );
    }
}

void recompute_mix() {
    const uint8_t regular_right_low = g_rumble.regular_right;
    const uint8_t regular_right_high = g_rumble.regular_right;
    const uint8_t regular_left_low = g_rumble.regular_left;
    const uint8_t regular_left_high = g_rumble.regular_left;

    const uint8_t right_low = mix_peak(regular_right_low, g_rumble.haptic_right_low);
    const uint8_t right_high = mix_peak(regular_right_high, g_rumble.haptic_right_high);
    const uint8_t left_low = mix_peak(regular_left_low, g_rumble.haptic_left_low);
    const uint8_t left_high = mix_peak(regular_left_high, g_rumble.haptic_left_high);

    g_rumble.mixed_right_low = right_low;
    g_rumble.mixed_right_high = right_high;
    g_rumble.mixed_left_low = left_low;
    g_rumble.mixed_left_high = left_high;

    const bool has_rumble = g_rumble.regular_right != 0 || g_rumble.regular_left != 0;
    const bool has_haptics =
        g_rumble.haptic_right_low != 0 || g_rumble.haptic_right_high != 0 ||
        g_rumble.haptic_left_low != 0 || g_rumble.haptic_left_high != 0;
    const bool haptics_dominant = has_haptics && (
        g_rumble.haptic_right_low >= regular_right_low ||
        g_rumble.haptic_right_high >= regular_right_high ||
        g_rumble.haptic_left_low >= regular_left_low ||
        g_rumble.haptic_left_high >= regular_left_high
    );
    if (has_rumble && has_haptics) {
        g_rumble.source = kSourceMixed;
    } else if (has_haptics) {
        g_rumble.source = kSourceHaptics;
    } else if (has_rumble) {
        g_rumble.source = kSourceRumble;
    } else {
        g_rumble.source = kSourceNone;
    }
    const bool audio_haptics_style = get_ns2pro_config().rumble_style == kRumbleStyleAudioHaptics;
    g_rumble.profile = (audio_haptics_style && haptics_dominant) ? kRumbleStyleAudioHaptics : kRumbleStyleDirect;
}

void reset_output_report(uint8_t out_report[64]) {
    std::memset(out_report, 0, kNs2ProOutputReportLen);
    out_report[0] = 0x02;
}

void reset_haptics_merge_state() {
    g_haptics_left_low_state = 0.0f;
    g_haptics_right_low_state = 0.0f;
}

} // namespace

void ns2pro_rumble_init() {
    g_rumble = RumbleState{};
    reset_haptics_merge_state();
    g_rumble.next_stream_at = get_absolute_time();
}

void ns2pro_rumble_on_ds5_output_report(const uint8_t *report, size_t len) {
    if (!report || len < 4 || report[0] != 0x02) {
        return;
    }
    ++g_rumble.input_count;

    SetStateData state{};
    const size_t payload_len = std::min(len - 1, sizeof(SetStateData));
    std::memcpy(&state, report + 1, payload_len);

    const bool rumble_enabled = state.UseRumbleNotHaptics ||
        state.EnableRumbleEmulation ||
        state.EnableImprovedRumbleEmulation;
    if (!rumble_enabled) {
        g_rumble.regular_right = 0;
        g_rumble.regular_left = 0;
    } else {
        g_rumble.regular_right = state.RumbleEmulationRight;
        g_rumble.regular_left = state.RumbleEmulationLeft;
    }

    recompute_mix();
    g_rumble.dirty = true;
    g_rumble.next_stream_at = get_absolute_time();
}

void ns2pro_rumble_on_haptics_audio(const int8_t *samples, size_t len) {
    if (!samples || len < 2) {
        return;
    }
    ++g_rumble.input_count;

    const float alpha = lowpass_alpha(kHapticsCrossoverHz, kHapticsSampleRateHz);
    float left_low_sum = 0.0f;
    float right_low_sum = 0.0f;
    float left_high_sum = 0.0f;
    float right_high_sum = 0.0f;
    float left_high_peak = 0.0f;
    float right_high_peak = 0.0f;
    size_t frames = 0;
    for (size_t i = 0; i + 1 < len; i += 2) {
        const float left = static_cast<float>(samples[i]) / 127.0f;
        const float right = static_cast<float>(samples[i + 1]) / 127.0f;

        g_haptics_left_low_state += alpha * (left - g_haptics_left_low_state);
        g_haptics_right_low_state += alpha * (right - g_haptics_right_low_state);

        const float left_low = g_haptics_left_low_state;
        const float right_low = g_haptics_right_low_state;
        const float left_high = left - left_low;
        const float right_high = right - right_low;

        left_low_sum += std::fabs(left_low);
        right_low_sum += std::fabs(right_low);
        left_high_sum += std::fabs(left_high);
        right_high_sum += std::fabs(right_high);
        left_high_peak = std::max(left_high_peak, std::fabs(left_high));
        right_high_peak = std::max(right_high_peak, std::fabs(right_high));
        ++frames;
    }

    if (frames == 0) {
        return;
    }

    const float left_low_strength = left_low_sum / static_cast<float>(frames);
    const float right_low_strength = right_low_sum / static_cast<float>(frames);
    const float left_high_strength = std::max(left_high_sum / static_cast<float>(frames), left_high_peak);
    const float right_high_strength = std::max(right_high_sum / static_cast<float>(frames), right_high_peak);
    g_rumble.haptic_left_low = map_easy_input_level(
        left_low_strength,
        kEasyInputLowGain,
        kEasyInputLowResponseCurve,
        kEasyInputNoiseGate
    );
    g_rumble.haptic_left_high = map_easy_input_level(
        left_high_strength,
        kEasyInputHighGain,
        kEasyInputHighResponseCurve,
        kEasyInputNoiseGate
    );
    g_rumble.haptic_right_low = map_easy_input_level(
        right_low_strength,
        kEasyInputLowGain,
        kEasyInputLowResponseCurve,
        kEasyInputNoiseGate
    );
    g_rumble.haptic_right_high = map_easy_input_level(
        right_high_strength,
        kEasyInputHighGain,
        kEasyInputHighResponseCurve,
        kEasyInputNoiseGate
    );

    recompute_mix();
    g_rumble.dirty = true;
    g_rumble.next_stream_at = get_absolute_time();
}

void ns2pro_rumble_set_direct(uint8_t right, uint8_t left) {
    ++g_rumble.input_count;
    g_rumble.regular_right = right;
    g_rumble.regular_left = left;
    g_rumble.haptic_right_low = 0;
    g_rumble.haptic_right_high = 0;
    g_rumble.haptic_left_low = 0;
    g_rumble.haptic_left_high = 0;
    recompute_mix();
    g_rumble.dirty = true;
    g_rumble.next_stream_at = get_absolute_time();
}

void ns2pro_rumble_set_direct_quad(uint8_t left_high, uint8_t left_low, uint8_t right_high, uint8_t right_low) {
    ++g_rumble.input_count;
    g_rumble.regular_right = 0;
    g_rumble.regular_left = 0;
    g_rumble.haptic_left_high = left_high;
    g_rumble.haptic_left_low = left_low;
    g_rumble.haptic_right_high = right_high;
    g_rumble.haptic_right_low = right_low;
    recompute_mix();
    g_rumble.dirty = true;
    g_rumble.next_stream_at = get_absolute_time();
}

bool ns2pro_rumble_build_output_report(uint8_t out_report[64]) {
    if (!out_report) {
        return false;
    }

    if (!g_rumble.dirty && g_rumble.source == kSourceNone) {
        return false;
    }

    reset_output_report(out_report);
    const uint8_t seq_tag = static_cast<uint8_t>(0x50u | (g_rumble.sequence & 0x0Fu));
    out_report[1] = seq_tag;
    out_report[17] = seq_tag;
    out_report[16] = 0x00;
    out_report[32] = 0x00;

    uint8_t left_frames[3][5]{};
    uint8_t right_frames[3][5]{};
    build_rumble_frames(
        g_rumble.mixed_left_low,
        g_rumble.mixed_left_high,
        g_rumble.profile == kRumbleStyleAudioHaptics,
        left_frames
    );
    build_rumble_frames(
        g_rumble.mixed_right_low,
        g_rumble.mixed_right_high,
        g_rumble.profile == kRumbleStyleAudioHaptics,
        right_frames
    );
    for (size_t i = 0; i < 3; ++i) {
        std::memcpy(out_report + 2 + i * 5, left_frames[i], 5);
        std::memcpy(out_report + 18 + i * 5, right_frames[i], 5);
    }
    std::memcpy(g_rumble.last_report_head, out_report, sizeof(g_rumble.last_report_head));

    g_rumble.sequence = static_cast<uint8_t>((g_rumble.sequence + 1u) & 0x0Fu);
    ++g_rumble.build_count;
    g_rumble.dirty = false;
    return true;
}

bool ns2pro_rumble_should_stream_output() {
    if (g_rumble.source == kSourceNone) {
        return false;
    }

    const absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, g_rumble.next_stream_at) > 0) {
        return false;
    }

    g_rumble.next_stream_at = delayed_by_us(now, 4000);
    g_rumble.dirty = true;
    return true;
}

void ns2pro_rumble_note_output_sent(bool usb_queued) {
    ++g_rumble.ble_sent_count;
    if (usb_queued) {
        ++g_rumble.usb_queued_count;
    }
}

void ns2pro_rumble_note_input_observed() {
    ++g_rumble.input_count;
}

void ns2pro_rumble_note_report_built() {
    ++g_rumble.build_count;
}

void ns2pro_rumble_note_output_attempt(bool success, uint8_t stage) {
    ++g_rumble.send_attempt_count;
    g_rumble.last_send_stage = stage;
    if (success) {
        ++g_rumble.send_success_count;
    } else {
        ++g_rumble.send_failure_count;
    }
}

void ns2pro_rumble_get_debug(Ns2ProRumbleDebug *out_debug) {
    if (!out_debug) {
        return;
    }
    out_debug->last_sequence = g_rumble.sequence;
    out_debug->last_mixed_low = std::max(g_rumble.mixed_right_low, g_rumble.mixed_left_low);
    out_debug->last_mixed_high = std::max(g_rumble.mixed_right_high, g_rumble.mixed_left_high);
    out_debug->last_source = g_rumble.source;
    out_debug->input_count = g_rumble.input_count;
    out_debug->build_count = g_rumble.build_count;
    out_debug->send_attempt_count = g_rumble.send_attempt_count;
    out_debug->send_success_count = g_rumble.send_success_count;
    out_debug->send_failure_count = g_rumble.send_failure_count;
    out_debug->last_send_stage = g_rumble.last_send_stage;
    out_debug->ble_sent_count = g_rumble.ble_sent_count;
    out_debug->usb_queued_count = g_rumble.usb_queued_count;
    std::memcpy(out_debug->last_report_head, g_rumble.last_report_head, sizeof(out_debug->last_report_head));
}
