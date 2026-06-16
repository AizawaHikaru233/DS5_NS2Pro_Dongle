#include "bridge_manager_config.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "config.h"
#include "ns2pro_config.h"
#include "button_mapping.h"

namespace {

constexpr uint8_t kBridgeManagerConfigVersion = 8;

float round_to_step(float value, float step) {
    return std::round(value / step) * step;
}

float clamp_float(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

uint8_t clamp_u8(int value, int min_value, int max_value) {
    return static_cast<uint8_t>(std::clamp(value, min_value, max_value));
}

float firmware_speaker_volume_to_manager(uint8_t value) {
    return -static_cast<float>(std::clamp<int>(value, 0, 100));
}

uint8_t manager_speaker_volume_to_firmware(float value) {
    const float clamped = clamp_float(value, -100.0f, 0.0f);
    return clamp_u8(static_cast<int>(std::lround(-clamped)), 0, 100);
}

BridgeManagerConfigBody build_manager_config() {
    const auto &cfg = get_config();
    const auto &ns2_cfg = get_ns2pro_config();

    BridgeManagerConfigBody body{};
    body.config_version = kBridgeManagerConfigVersion;
    body.ds5_haptics_gain = clamp_float(cfg.haptics_gain, 0.0f, 2.0f);
    body.speaker_volume = firmware_speaker_volume_to_manager(cfg.speaker_volume);
    body.inactive_time = cfg.inactive_time;
    body.disable_inactive_disconnect = cfg.disable_inactive_disconnect ? 1 : 0;
    body.disable_pico_led = cfg.disable_pico_led ? 1 : 0;
    body.polling_rate_mode = cfg.polling_rate_mode;
    body.haptics_buffer_length = cfg.audio_buffer_length;
    body.controller_mode = cfg.controller_mode;
    body.ns2pro_rumble_gain = clamp_float(ns2_cfg.rumble_gain, 0.0f, 2.0f);
    body.ns2pro_rumble_style = ns2_cfg.rumble_style;
    body.ns2pro_ble_has_target = ns2_cfg.ble_has_target ? 1 : 0;
    body.ns2pro_ble_address_type = ns2_cfg.ble_address_type;
    std::memcpy(body.ns2pro_ble_address, ns2_cfg.ble_address, sizeof(body.ns2pro_ble_address));
    body.ds5_left_stick_deadzone_percent = cfg.ds5_left_stick_deadzone_percent;
    body.ds5_right_stick_deadzone_percent = cfg.ds5_right_stick_deadzone_percent;
    body.ns2pro_left_stick_deadzone_percent = ns2_cfg.left_stick_deadzone_percent;
    body.ns2pro_right_stick_deadzone_percent = ns2_cfg.right_stick_deadzone_percent;
    body.ns2pro_auto_stick_center = ns2_cfg.auto_stick_center ? 1 : 0;
    return body;
}

void apply_manager_config(const BridgeManagerConfigBody &body) {
    auto cfg = get_config();
    cfg.config_version = get_config().config_version;
    cfg.haptics_gain = clamp_float(body.ds5_haptics_gain, 0.0f, 2.0f);
    const uint8_t mapped_speaker_volume = manager_speaker_volume_to_firmware(body.speaker_volume);
    cfg.speaker_volume = mapped_speaker_volume;
    cfg.headset_volume = mapped_speaker_volume;
    cfg.sync_spk_headset_volume = 1;
    cfg.inactive_time = clamp_u8(body.inactive_time, 5, 60);
    cfg.disable_inactive_disconnect = body.disable_inactive_disconnect ? 1 : 0;
    cfg.disable_pico_led = body.disable_pico_led ? 1 : 0;
    cfg.polling_rate_mode = clamp_u8(body.polling_rate_mode, 0, 2);
    cfg.audio_buffer_length = clamp_u8(body.haptics_buffer_length, 16, 128);
    cfg.controller_mode = clamp_u8(body.controller_mode, 0, 2);
    cfg.ds5_left_stick_deadzone_percent = clamp_u8(body.ds5_left_stick_deadzone_percent, 0, 30);
    cfg.ds5_right_stick_deadzone_percent = clamp_u8(body.ds5_right_stick_deadzone_percent, 0, 30);
    set_config(cfg);

    auto ns2_cfg = get_ns2pro_config();
    ns2_cfg.config_version = get_ns2pro_config().config_version;
    ns2_cfg.rumble_gain = round_to_step(clamp_float(body.ns2pro_rumble_gain, 0.0f, 2.0f), 0.01f);
    ns2_cfg.rumble_style = clamp_u8(body.ns2pro_rumble_style, 0, 1);
    ns2_cfg.ble_has_target = body.ns2pro_ble_has_target ? 1 : 0;
    ns2_cfg.ble_address_type = clamp_u8(body.ns2pro_ble_address_type, 0, 1);
    std::memcpy(ns2_cfg.ble_address, body.ns2pro_ble_address, sizeof(ns2_cfg.ble_address));
    ns2_cfg.left_stick_deadzone_percent = clamp_u8(body.ns2pro_left_stick_deadzone_percent, 0, 30);
    ns2_cfg.right_stick_deadzone_percent = clamp_u8(body.ns2pro_right_stick_deadzone_percent, 0, 30);
    ns2_cfg.auto_stick_center = body.ns2pro_auto_stick_center ? 1 : 0;
    set_ns2pro_config(ns2_cfg);
}

} // namespace

size_t bridge_manager_config_get(uint8_t *buffer, uint16_t reqlen) {
    if (!buffer || reqlen == 0) {
        return 0;
    }

    const BridgeManagerConfigBody body = build_manager_config();
    const size_t copy_len = std::min(sizeof(body), static_cast<size_t>(reqlen));
    std::memcpy(buffer, &body, copy_len);
    return copy_len;
}

void bridge_manager_config_set(const uint8_t *buffer, uint16_t len) {
    if (!buffer || len == 0) {
        return;
    }

    BridgeManagerConfigBody body = build_manager_config();
    const size_t copy_len = std::min(sizeof(body), static_cast<size_t>(len));
    std::memcpy(&body, buffer, copy_len);
    apply_manager_config(body);
}

bool bridge_manager_config_save() {
    const bool config_saved = config_save();
    const bool ns2pro_config_saved = ns2pro_config_save();
    const bool button_mapping_saved = button_mapping_save();
    return config_saved && ns2pro_config_saved && button_mapping_saved;
}
