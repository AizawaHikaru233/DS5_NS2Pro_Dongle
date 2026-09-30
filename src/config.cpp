//
// Created by awalol on 2026/5/4.
//

#include "config.h"

#include <cmath>
#include <cstring>

#include "state_mgr.h"
#include "utils.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico_led.h"

#if __has_include("pico/btstack_flash_bank.h")
#include "pico/btstack_flash_bank.h"
#endif

constexpr uint32_t CONFIG_MAGIC = 0x66ccff00;
constexpr uint16_t CONFIG_VERSION = 3;
// Flash map (from the end of flash, one 4 KiB sector each):
//   -1 sector : main config (this file)
//   -2, -3    : BTstack BLE bond / link-key TLV store (pico_btstack_flash_bank)
//   -4 sector : NS2Pro config (ns2pro_config.cpp)
//   -5 sector : button mapping (button_mapping.cpp)
// The last sector must stay free of the BLE bond store, which is why the bond
// bank sits two sectors lower and the other stores were moved below it.
constexpr uint32_t CONFIG_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static Config config{};
bool is_dse = false;

// 编译期保护
// 判断Config结构体是否能放进flash 256bytes
static_assert(sizeof(Config) <= FLASH_PAGE_SIZE);
// 配置区起始地址必须按 flash sector 对齐。
static_assert(CONFIG_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);
#if defined(PICO_FLASH_BANK_STORAGE_OFFSET) && defined(PICO_FLASH_BANK_TOTAL_SIZE)
static_assert(
    CONFIG_FLASH_OFFSET + FLASH_SECTOR_SIZE <= PICO_FLASH_BANK_STORAGE_OFFSET ||
        CONFIG_FLASH_OFFSET >= PICO_FLASH_BANK_STORAGE_OFFSET + PICO_FLASH_BANK_TOTAL_SIZE,
    "Main config sector must not overlap the BTstack BLE flash bank"
);
#endif

uint32_t calc_config_crc(const Config &con) {
    return crc32(reinterpret_cast<const uint8_t *>(&con.body), sizeof(Config_body));
}

static void apply_runtime_config() {
    pico_led_restore_configured_state();
    set_volume(config.body.speaker_volume, config.body.headset_volume);
    set_gain(config.body.speaker_gain);
}

const Config *flash_config() {
    return reinterpret_cast<const Config *>(XIP_BASE + CONFIG_FLASH_OFFSET);
}

void config_valid() {
    // valid config and set default value
    if (config.magic != CONFIG_MAGIC || config.size != sizeof(Config_body)) {
        printf("[Config] Config Magic Header is invalid\n");
        printf("[Config] Config Body size is invalid\n");
        config_default();
        return;
    }
    auto body = &config.body;
    if (std::isnan(body->haptics_gain) || body->haptics_gain < 0.0f || body->haptics_gain > 2.0f) {
        body->haptics_gain = 1.0f;
        printf("[Config] Haptics Gain value is invalid\n");
    }
    if (body->speaker_volume < 0 || body->speaker_volume > 127) {
        body->speaker_volume = 100;
        printf("[Config] Speaker Volume is invalid\n");
    }
    if (body->headset_volume < 0 || body->headset_volume > 127) {
        body->headset_volume = 100;
        printf("[Config] Headset Volume is invalid\n");
    }
    if (body->sync_spk_headset_volume > 1) {
        body->sync_spk_headset_volume = 0;
        printf("[Config] sync_spk_headset_volume is invalid\n");
    }
    if (body->speaker_gain < 0 || body->speaker_gain > 7) {
        body->speaker_gain = 2;
        printf("[Config] speaker_gain is invalid\n");
    }
    if (body->inactive_time < 5 || body->inactive_time > 60) {
        body->inactive_time = 30;
        printf("[Config] Inactive time is invalid\n");
    }
    if (body->disable_inactive_disconnect > 1) {
        body->disable_inactive_disconnect = 0;
        printf("[Config] disable_auto_disconnect is invalid\n");
    }
    if (body->disable_pico_led > 1) {
        body->disable_pico_led = 0;
        printf("[Config] disable_pico_led is invalid\n");
    }
    if (body->polling_rate_mode > 2) {
        body->polling_rate_mode = 0;
        printf("[Config] polling_rate_mode is invalid\n");
    }
    if (body->audio_buffer_length < 16 || body->audio_buffer_length > 128) {
        body->audio_buffer_length = 64;
        printf("[Config] haptics_buffer_length is invalid\n");
    }
    if (body->controller_mode > 2) {
        body->controller_mode = 2;
        printf("[Config] controller_mode is invalid\n");
    }
    if (body->config_version > CONFIG_VERSION) {
        body->config_version = CONFIG_VERSION;
        printf("[Config] Warning: Config version is newer than firmware, clamping\n");
    } else if (body->config_version != CONFIG_VERSION) {
        body->config_version = CONFIG_VERSION;
        printf("[Config] Warning: Config may breaking change\n");
    }
    if (body->lock_volume > 1) {
        body->lock_volume = 0;
        printf("[Config] lock_volume is invalid\n");
    }
    if (body->disable_usb_sn > 1) {
        body->disable_usb_sn = 0;
        printf("[Config] Warning: disable_usb_sn is invalid\n");
    }
    if (body->ps_shortcut_enabled > 1) {
        body->ps_shortcut_enabled = 0;
        printf("[Config] ps_shortcut_enabled is invalid\n");
    }
    if (body->ds5_left_stick_deadzone_percent > 30) {
        body->ds5_left_stick_deadzone_percent = 3;
        printf("[Config] ds5_left_stick_deadzone_percent is invalid\n");
    }
    if (body->ds5_right_stick_deadzone_percent > 30) {
        body->ds5_right_stick_deadzone_percent = 0;
        printf("[Config] ds5_right_stick_deadzone_percent is invalid\n");
    }
}

void config_default() {
    memset(&config, 0, sizeof(config));
    config.magic = CONFIG_MAGIC;
    config.size = sizeof(Config_body);
    config.body.config_version = CONFIG_VERSION;
    config.body.haptics_gain = 1.0f;
    config.body.speaker_volume = 0;
    config.body.headset_volume = 0;
    config.body.sync_spk_headset_volume = 1;
    config.body.speaker_gain = 2;
    config.body.inactive_time = 30;
    config.body.disable_inactive_disconnect = 0;
    config.body.disable_pico_led = 0;
    config.body.polling_rate_mode = 0;
    config.body.audio_buffer_length = 64;
    config.body.controller_mode = 2;
    config.body.lock_volume = 0;
    config.body.disable_usb_sn = 0;
    config.body.ps_shortcut_enabled = 0;
    config.body.ds5_left_stick_deadzone_percent = 3;
    config.body.ds5_right_stick_deadzone_percent = 0;
}

void config_load() {
    memcpy(&config, flash_config(), sizeof(Config));

    config_valid();
}

bool config_save() {
    config.crc32 = calc_config_crc(config);
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &config, sizeof(Config));

    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, page, sizeof(page));
    restore_interrupts(interrupts);

    Config verify{};
    memcpy(&verify, flash_config(), sizeof(verify));
    const auto verify_crc32 = calc_config_crc(verify);
    if (verify_crc32 == config.crc32) {
        printf("[Config] Config write flash verify success\n");
        return true;
    }
    printf("[Config] Config write flash verify failed\n");
    return false;
}

Config_body& get_config() {
    return config.body;
}

void set_config(const uint8_t *new_config, const uint16_t len) {
    const auto copy_len = len < sizeof(Config_body) ? len : sizeof(Config_body);
    memcpy(&config.body, new_config, copy_len);
    config_valid();
    apply_runtime_config();
}

void set_config(const Config_body &new_config) {
    config.body = new_config;
    config_valid();
    apply_runtime_config();
}
