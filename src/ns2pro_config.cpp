#include "ns2pro_config.h"

#include <cmath>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "ns2pro_translator.h"
#include "utils.h"

#if __has_include("pico/btstack_flash_bank.h")
#include "pico/btstack_flash_bank.h"
#endif

namespace {

constexpr uint32_t NS2PRO_CONFIG_MAGIC = 0x4e325043;
constexpr uint16_t NS2PRO_CONFIG_VERSION = 3;
// Flash map (from the end of flash, one 4 KiB sector each):
//   -1 sector : main config (config.cpp)
//   -2, -3    : BTstack BLE bond / link-key TLV store (pico_btstack_flash_bank)
//   -4 sector : NS2Pro config (this file)
//   -5 sector : button mapping (button_mapping.cpp)
// The BLE bond store used to share -2/-3 with this file and the button mapping,
// so every bank rotation in the bond log silently erased them.
constexpr uint32_t NS2PRO_CONFIG_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - (4 * FLASH_SECTOR_SIZE);
// Where this config lived before the collision was fixed, read once for migration.
constexpr uint32_t NS2PRO_CONFIG_LEGACY_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - (2 * FLASH_SECTOR_SIZE);

Ns2ProConfig config{};

static_assert(sizeof(Ns2ProConfig) <= FLASH_PAGE_SIZE);
static_assert(NS2PRO_CONFIG_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);
#if defined(PICO_FLASH_BANK_STORAGE_OFFSET) && defined(PICO_FLASH_BANK_TOTAL_SIZE)
static_assert(
    NS2PRO_CONFIG_FLASH_OFFSET + FLASH_SECTOR_SIZE <= PICO_FLASH_BANK_STORAGE_OFFSET ||
        NS2PRO_CONFIG_FLASH_OFFSET >= PICO_FLASH_BANK_STORAGE_OFFSET + PICO_FLASH_BANK_TOTAL_SIZE,
    "NS2Pro config sector must not overlap the BTstack BLE flash bank"
);
#endif

uint32_t calc_config_crc(const Ns2ProConfig &value) {
    return crc32(reinterpret_cast<const uint8_t *>(&value.body), sizeof(Ns2ProConfigBody));
}

const Ns2ProConfig *flash_config() {
    return reinterpret_cast<const Ns2ProConfig *>(XIP_BASE + NS2PRO_CONFIG_FLASH_OFFSET);
}

const Ns2ProConfig *legacy_flash_config() {
    return reinterpret_cast<const Ns2ProConfig *>(XIP_BASE + NS2PRO_CONFIG_LEGACY_FLASH_OFFSET);
}

bool storage_is_usable(const Ns2ProConfig &value) {
    return value.magic == NS2PRO_CONFIG_MAGIC && value.size == sizeof(Ns2ProConfigBody);
}

uint8_t clamp_percent(uint8_t value, uint8_t fallback) {
    return value <= 30 ? value : fallback;
}

} // namespace

void ns2pro_config_default() {
    memset(&config, 0, sizeof(config));
    config.magic = NS2PRO_CONFIG_MAGIC;
    config.size = sizeof(Ns2ProConfigBody);
    config.body.config_version = NS2PRO_CONFIG_VERSION;
    config.body.rumble_gain = 1.0f;
    config.body.rumble_style = 1;
    config.body.ble_has_target = 0;
    config.body.ble_address_type = 0;
    memset(config.body.ble_address, 0, sizeof(config.body.ble_address));
    config.body.left_stick_deadzone_percent = 3;
    config.body.right_stick_deadzone_percent = 0;
    config.body.auto_stick_center = 1;
    config.body.center_lx = 2048;
    config.body.center_ly = 2048;
    config.body.center_rx = 2048;
    config.body.center_ry = 2048;
    config.body.gyro_invert_y = 1;
    // Gyro zero-bias centering is manual only; nothing re-centers it on its own.
    config.body.auto_gyro_center = 0;
    config.body.gyro_center_x = 0;
    config.body.gyro_center_y = 0;
    config.body.gyro_center_z = 0;
    ns2pro_reset_runtime_calibration_from_config();
}

void ns2pro_config_valid() {
    if (config.magic != NS2PRO_CONFIG_MAGIC || config.size != sizeof(Ns2ProConfigBody)) {
        ns2pro_config_default();
        return;
    }

    auto &body = config.body;
    if (body.config_version > NS2PRO_CONFIG_VERSION) {
        body.config_version = NS2PRO_CONFIG_VERSION;
    } else if (body.config_version < 2) {
        body.config_version = NS2PRO_CONFIG_VERSION;
        body.auto_gyro_center = 0;
        body.gyro_center_x = 0;
        body.gyro_center_y = 0;
        body.gyro_center_z = 0;
    } else if (body.config_version != NS2PRO_CONFIG_VERSION) {
        body.config_version = NS2PRO_CONFIG_VERSION;
    }
    if (std::isnan(body.rumble_gain) || body.rumble_gain < 0.0f || body.rumble_gain > 2.0f) {
        body.rumble_gain = 1.0f;
    }
    if (body.rumble_style > 1) {
        body.rumble_style = 1;
    }
    if (body.ble_has_target > 1) {
        body.ble_has_target = 0;
    }
    if (body.ble_address_type > 1) {
        body.ble_address_type = 0;
    }
    body.left_stick_deadzone_percent = clamp_percent(body.left_stick_deadzone_percent, 3);
    body.right_stick_deadzone_percent = clamp_percent(body.right_stick_deadzone_percent, 0);
    if (body.auto_stick_center > 1) {
        body.auto_stick_center = 1;
    }
    if (body.center_lx < 0 || body.center_lx > 4095) body.center_lx = 2048;
    if (body.center_ly < 0 || body.center_ly > 4095) body.center_ly = 2048;
    if (body.center_rx < 0 || body.center_rx > 4095) body.center_rx = 2048;
    if (body.center_ry < 0 || body.center_ry > 4095) body.center_ry = 2048;
    if (body.gyro_invert_y > 1) {
        body.gyro_invert_y = 1;
    }
    // Legacy flag: gyro centering is manual only now, keep it sane but unused.
    if (body.auto_gyro_center > 1) {
        body.auto_gyro_center = 0;
    }
    // Manual gyro calibration writes these; reject flash garbage.
    constexpr int16_t kGyroCenterLimit = 2000;
    if (body.gyro_center_x < -kGyroCenterLimit || body.gyro_center_x > kGyroCenterLimit) body.gyro_center_x = 0;
    if (body.gyro_center_y < -kGyroCenterLimit || body.gyro_center_y > kGyroCenterLimit) body.gyro_center_y = 0;
    if (body.gyro_center_z < -kGyroCenterLimit || body.gyro_center_z > kGyroCenterLimit) body.gyro_center_z = 0;
}

void ns2pro_config_load() {
    memcpy(&config, flash_config(), sizeof(config));
    if (!storage_is_usable(config)) {
        // Best effort migration from the pre-fix location so an existing BLE
        // target address / calibration is not lost after the flash map change.
        const Ns2ProConfig *legacy = legacy_flash_config();
        if (legacy && storage_is_usable(*legacy)) {
            memcpy(&config, legacy, sizeof(config));
            ns2pro_config_valid();
            ns2pro_config_save();
            ns2pro_reset_runtime_calibration_from_config();
            return;
        }
    }
    ns2pro_config_valid();
    ns2pro_reset_runtime_calibration_from_config();
}

bool ns2pro_config_save() {
    config.crc32 = calc_config_crc(config);
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &config, sizeof(config));

    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(NS2PRO_CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(NS2PRO_CONFIG_FLASH_OFFSET, page, sizeof(page));
    restore_interrupts(interrupts);

    Ns2ProConfig verify{};
    memcpy(&verify, flash_config(), sizeof(verify));
    return calc_config_crc(verify) == config.crc32;
}

Ns2ProConfigBody &get_ns2pro_config() {
    return config.body;
}

void set_ns2pro_config(const uint8_t *new_config, uint16_t len) {
    const auto copy_len = len < sizeof(Ns2ProConfigBody) ? len : sizeof(Ns2ProConfigBody);
    memcpy(&config.body, new_config, copy_len);
    ns2pro_config_valid();
    ns2pro_reset_runtime_calibration_from_config();
}

void set_ns2pro_config(const Ns2ProConfigBody &new_config) {
    config.body = new_config;
    ns2pro_config_valid();
    ns2pro_reset_runtime_calibration_from_config();
}
