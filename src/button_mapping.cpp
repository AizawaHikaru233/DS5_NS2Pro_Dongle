#include "button_mapping.h"

#include <algorithm>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "utils.h"

namespace {

constexpr uint32_t kButtonMappingMagic = 0x42544d50;
constexpr uint16_t kButtonMappingVersion = 2;
constexpr uint32_t kButtonMappingFlashOffset = PICO_FLASH_SIZE_BYTES - (3 * FLASH_SECTOR_SIZE);

struct __attribute__((packed)) ButtonMappingBody {
    uint16_t config_version;
    uint8_t ds5_targets[kDs5ButtonInputCount];
    uint8_t ns2pro_targets[kNs2ProButtonInputCount];
};

struct __attribute__((packed)) ButtonMappingStorage {
    uint32_t magic;
    uint32_t crc32;
    uint16_t size;
    ButtonMappingBody body;
};

ButtonMappingStorage g_storage{};

static_assert(sizeof(ButtonMappingStorage) <= FLASH_PAGE_SIZE);
static_assert(kButtonMappingFlashOffset % FLASH_SECTOR_SIZE == 0);

uint32_t calc_crc(const ButtonMappingStorage &storage) {
    return crc32(reinterpret_cast<const uint8_t *>(&storage.body), sizeof(storage.body));
}

const ButtonMappingStorage *flash_storage() {
    return reinterpret_cast<const ButtonMappingStorage *>(XIP_BASE + kButtonMappingFlashOffset);
}

uint8_t default_ds5_target(uint8_t input) {
    return input < kDs5ButtonInputCount ? input : kButtonMappingTargetNone;
}

uint8_t default_ns2pro_target(uint8_t input) {
    switch (input) {
        case kNs2ProButtonInputUp: return kButtonMappingTargetUp;
        case kNs2ProButtonInputRight: return kButtonMappingTargetRight;
        case kNs2ProButtonInputDown: return kButtonMappingTargetDown;
        case kNs2ProButtonInputLeft: return kButtonMappingTargetLeft;
        case kNs2ProButtonInputY: return kButtonMappingTargetSquare;
        case kNs2ProButtonInputB: return kButtonMappingTargetCross;
        case kNs2ProButtonInputA: return kButtonMappingTargetCircle;
        case kNs2ProButtonInputX: return kButtonMappingTargetTriangle;
        case kNs2ProButtonInputL: return kButtonMappingTargetL1;
        case kNs2ProButtonInputR: return kButtonMappingTargetR1;
        case kNs2ProButtonInputZL: return kButtonMappingTargetL2;
        case kNs2ProButtonInputZR: return kButtonMappingTargetR2;
        case kNs2ProButtonInputMinus: return kButtonMappingTargetCreate;
        case kNs2ProButtonInputPlus: return kButtonMappingTargetOptions;
        case kNs2ProButtonInputL3: return kButtonMappingTargetL3;
        case kNs2ProButtonInputR3: return kButtonMappingTargetR3;
        case kNs2ProButtonInputHome: return kButtonMappingTargetPs;
        case kNs2ProButtonInputCapture: return kButtonMappingTargetTouchpad;
        case kNs2ProButtonInputGL: return kButtonMappingTargetNone;
        case kNs2ProButtonInputGR: return kButtonMappingTargetNone;
        default: return kButtonMappingTargetNone;
    }
}

bool target_is_valid(uint8_t target) {
    return target == kButtonMappingTargetNone || target < kButtonMappingTargetCount;
}

void validate_targets(uint8_t *targets, size_t count, uint8_t (*default_target)(uint8_t)) {
    for (size_t i = 0; i < count; ++i) {
        if (!target_is_valid(targets[i])) {
            targets[i] = default_target(static_cast<uint8_t>(i));
        }
    }
}

void apply_defaults(ButtonMappingBody *body) {
    body->config_version = kButtonMappingVersion;
    for (uint8_t i = 0; i < kDs5ButtonInputCount; ++i) {
        body->ds5_targets[i] = default_ds5_target(i);
    }
    for (uint8_t i = 0; i < kNs2ProButtonInputCount; ++i) {
        body->ns2pro_targets[i] = default_ns2pro_target(i);
    }
}

void validate_storage() {
    if (g_storage.magic != kButtonMappingMagic || g_storage.size != sizeof(ButtonMappingBody)) {
        button_mapping_default();
        return;
    }

    if (g_storage.body.config_version < 2) {
        if (g_storage.body.ns2pro_targets[kNs2ProButtonInputGL] == kButtonMappingTargetLeftPaddle) {
            g_storage.body.ns2pro_targets[kNs2ProButtonInputGL] = kButtonMappingTargetNone;
        }
        if (g_storage.body.ns2pro_targets[kNs2ProButtonInputGR] == kButtonMappingTargetRightPaddle) {
            g_storage.body.ns2pro_targets[kNs2ProButtonInputGR] = kButtonMappingTargetNone;
        }
    }
    g_storage.body.config_version = kButtonMappingVersion;
    validate_targets(g_storage.body.ds5_targets, kDs5ButtonInputCount, default_ds5_target);
    validate_targets(g_storage.body.ns2pro_targets, kNs2ProButtonInputCount, default_ns2pro_target);
}

uint8_t decode_hat_bits(uint8_t hat) {
    switch (hat & 0x0f) {
        case 0: return 1u << kButtonMappingTargetUp;
        case 1: return (1u << kButtonMappingTargetUp) | (1u << kButtonMappingTargetRight);
        case 2: return 1u << kButtonMappingTargetRight;
        case 3: return (1u << kButtonMappingTargetRight) | (1u << kButtonMappingTargetDown);
        case 4: return 1u << kButtonMappingTargetDown;
        case 5: return (1u << kButtonMappingTargetDown) | (1u << kButtonMappingTargetLeft);
        case 6: return 1u << kButtonMappingTargetLeft;
        case 7: return (1u << kButtonMappingTargetLeft) | (1u << kButtonMappingTargetUp);
        default: return 0;
    }
}

uint8_t encode_hat(uint32_t buttons) {
    const bool up = (buttons & (1u << kButtonMappingTargetUp)) != 0;
    const bool right = (buttons & (1u << kButtonMappingTargetRight)) != 0;
    const bool down = (buttons & (1u << kButtonMappingTargetDown)) != 0;
    const bool left = (buttons & (1u << kButtonMappingTargetLeft)) != 0;

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

uint32_t build_mapped_mask(const bool *pressed, size_t count, const uint8_t *targets) {
    uint32_t mapped = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!pressed[i]) {
            continue;
        }
        const uint8_t target = targets[i];
        if (target < kButtonMappingTargetCount) {
            mapped |= 1u << target;
        }
    }
    return mapped;
}

uint8_t mapped_trigger_value(
    const bool *pressed,
    const uint8_t *targets,
    size_t count,
    uint8_t trigger_target,
    size_t analog_input_index,
    uint8_t analog_value
) {
    uint8_t value = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!pressed[i] || targets[i] != trigger_target) {
            continue;
        }
        value = std::max<uint8_t>(value, i == analog_input_index ? analog_value : 0xff);
    }
    return value;
}

} // namespace

void button_mapping_default() {
    memset(&g_storage, 0, sizeof(g_storage));
    g_storage.magic = kButtonMappingMagic;
    g_storage.size = sizeof(ButtonMappingBody);
    apply_defaults(&g_storage.body);
}

void button_mapping_load() {
    memcpy(&g_storage, flash_storage(), sizeof(g_storage));
    validate_storage();
}

bool button_mapping_save() {
    g_storage.crc32 = calc_crc(g_storage);
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &g_storage, sizeof(g_storage));

    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(kButtonMappingFlashOffset, FLASH_SECTOR_SIZE);
    flash_range_program(kButtonMappingFlashOffset, page, sizeof(page));
    restore_interrupts(interrupts);

    ButtonMappingStorage verify{};
    memcpy(&verify, flash_storage(), sizeof(verify));
    return calc_crc(verify) == g_storage.crc32;
}

size_t button_mapping_get_ds5(uint8_t *buffer, size_t reqlen) {
    if (!buffer || reqlen == 0) {
        return 0;
    }
    const size_t copy_len = std::min(reqlen, static_cast<size_t>(kDs5ButtonInputCount));
    memcpy(buffer, g_storage.body.ds5_targets, copy_len);
    return copy_len;
}

size_t button_mapping_get_ns2pro(uint8_t *buffer, size_t reqlen) {
    if (!buffer || reqlen == 0) {
        return 0;
    }
    const size_t copy_len = std::min(reqlen, static_cast<size_t>(kNs2ProButtonInputCount));
    memcpy(buffer, g_storage.body.ns2pro_targets, copy_len);
    return copy_len;
}

void button_mapping_set_ds5(const uint8_t *buffer, size_t len) {
    if (!buffer || len == 0) {
        return;
    }
    const size_t copy_len = std::min(len, static_cast<size_t>(kDs5ButtonInputCount));
    memcpy(g_storage.body.ds5_targets, buffer, copy_len);
    validate_targets(g_storage.body.ds5_targets, kDs5ButtonInputCount, default_ds5_target);
}

void button_mapping_set_ns2pro(const uint8_t *buffer, size_t len) {
    if (!buffer || len == 0) {
        return;
    }
    const size_t copy_len = std::min(len, static_cast<size_t>(kNs2ProButtonInputCount));
    memcpy(g_storage.body.ns2pro_targets, buffer, copy_len);
    validate_targets(g_storage.body.ns2pro_targets, kNs2ProButtonInputCount, default_ns2pro_target);
}

void button_mapping_apply_ds5(uint8_t *report63, size_t len) {
    if (!report63 || len < 10) {
        return;
    }

    bool pressed[kDs5ButtonInputCount]{};
    const uint8_t buttons0 = report63[7];
    const uint8_t buttons1 = report63[8];
    const uint8_t buttons2 = report63[9];
    const uint8_t left_trigger = report63[4];
    const uint8_t right_trigger = report63[5];
    const uint8_t hat_bits = decode_hat_bits(buttons0 & 0x0f);

    pressed[kDs5ButtonInputUp] = (hat_bits & (1u << kButtonMappingTargetUp)) != 0;
    pressed[kDs5ButtonInputRight] = (hat_bits & (1u << kButtonMappingTargetRight)) != 0;
    pressed[kDs5ButtonInputDown] = (hat_bits & (1u << kButtonMappingTargetDown)) != 0;
    pressed[kDs5ButtonInputLeft] = (hat_bits & (1u << kButtonMappingTargetLeft)) != 0;
    pressed[kDs5ButtonInputSquare] = (buttons0 & (1u << 4)) != 0;
    pressed[kDs5ButtonInputCross] = (buttons0 & (1u << 5)) != 0;
    pressed[kDs5ButtonInputCircle] = (buttons0 & (1u << 6)) != 0;
    pressed[kDs5ButtonInputTriangle] = (buttons0 & (1u << 7)) != 0;
    pressed[kDs5ButtonInputL1] = (buttons1 & (1u << 0)) != 0;
    pressed[kDs5ButtonInputR1] = (buttons1 & (1u << 1)) != 0;
    pressed[kDs5ButtonInputL2] = left_trigger > 0 || (buttons1 & (1u << 2)) != 0;
    pressed[kDs5ButtonInputR2] = right_trigger > 0 || (buttons1 & (1u << 3)) != 0;
    pressed[kDs5ButtonInputCreate] = (buttons1 & (1u << 4)) != 0;
    pressed[kDs5ButtonInputOptions] = (buttons1 & (1u << 5)) != 0;
    pressed[kDs5ButtonInputL3] = (buttons1 & (1u << 6)) != 0;
    pressed[kDs5ButtonInputR3] = (buttons1 & (1u << 7)) != 0;
    pressed[kDs5ButtonInputPs] = (buttons2 & (1u << 0)) != 0;
    pressed[kDs5ButtonInputTouchpad] = (buttons2 & (1u << 1)) != 0;
    pressed[kDs5ButtonInputMute] = (buttons2 & (1u << 2)) != 0;
    pressed[kDs5ButtonInputLeftFunction] = (buttons2 & (1u << 4)) != 0;
    pressed[kDs5ButtonInputRightFunction] = (buttons2 & (1u << 5)) != 0;
    pressed[kDs5ButtonInputLeftPaddle] = (buttons2 & (1u << 6)) != 0;
    pressed[kDs5ButtonInputRightPaddle] = (buttons2 & (1u << 7)) != 0;

    const uint32_t mapped = build_mapped_mask(pressed, kDs5ButtonInputCount, g_storage.body.ds5_targets);

    report63[4] = mapped_trigger_value(
        pressed,
        g_storage.body.ds5_targets,
        kDs5ButtonInputCount,
        kButtonMappingTargetL2,
        kDs5ButtonInputL2,
        left_trigger
    );
    report63[5] = mapped_trigger_value(
        pressed,
        g_storage.body.ds5_targets,
        kDs5ButtonInputCount,
        kButtonMappingTargetR2,
        kDs5ButtonInputR2,
        right_trigger
    );

    report63[7] = encode_hat(mapped);
    if (mapped & (1u << kButtonMappingTargetSquare)) report63[7] |= 1u << 4;
    if (mapped & (1u << kButtonMappingTargetCross)) report63[7] |= 1u << 5;
    if (mapped & (1u << kButtonMappingTargetCircle)) report63[7] |= 1u << 6;
    if (mapped & (1u << kButtonMappingTargetTriangle)) report63[7] |= 1u << 7;

    report63[8] = 0;
    if (mapped & (1u << kButtonMappingTargetL1)) report63[8] |= 1u << 0;
    if (mapped & (1u << kButtonMappingTargetR1)) report63[8] |= 1u << 1;
    if (mapped & (1u << kButtonMappingTargetL2)) report63[8] |= 1u << 2;
    if (mapped & (1u << kButtonMappingTargetR2)) report63[8] |= 1u << 3;
    if (mapped & (1u << kButtonMappingTargetCreate)) report63[8] |= 1u << 4;
    if (mapped & (1u << kButtonMappingTargetOptions)) report63[8] |= 1u << 5;
    if (mapped & (1u << kButtonMappingTargetL3)) report63[8] |= 1u << 6;
    if (mapped & (1u << kButtonMappingTargetR3)) report63[8] |= 1u << 7;

    report63[9] = 0;
    if (mapped & (1u << kButtonMappingTargetPs)) report63[9] |= 1u << 0;
    if (mapped & (1u << kButtonMappingTargetTouchpad)) report63[9] |= 1u << 1;
    if (mapped & (1u << kButtonMappingTargetMute)) report63[9] |= 1u << 2;
    if (mapped & (1u << kButtonMappingTargetLeftFunction)) report63[9] |= 1u << 4;
    if (mapped & (1u << kButtonMappingTargetRightFunction)) report63[9] |= 1u << 5;
    if (mapped & (1u << kButtonMappingTargetLeftPaddle)) report63[9] |= 1u << 6;
    if (mapped & (1u << kButtonMappingTargetRightPaddle)) report63[9] |= 1u << 7;
}

uint32_t button_mapping_apply_ns2pro(uint32_t physical_buttons) {
    uint32_t mapped = 0;
    for (uint8_t input = 0; input < kNs2ProButtonInputCount; ++input) {
        if ((physical_buttons & (1u << input)) == 0) {
            continue;
        }
        const uint8_t target = g_storage.body.ns2pro_targets[input];
        if (target < kButtonMappingTargetCount) {
            mapped |= 1u << target;
        }
    }
    return mapped;
}
