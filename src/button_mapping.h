#ifndef DS5_BRIDGE_BUTTON_MAPPING_H
#define DS5_BRIDGE_BUTTON_MAPPING_H

#include <cstddef>
#include <cstdint>

constexpr uint8_t kButtonMappingTargetUp = 0;
constexpr uint8_t kButtonMappingTargetRight = 1;
constexpr uint8_t kButtonMappingTargetDown = 2;
constexpr uint8_t kButtonMappingTargetLeft = 3;
constexpr uint8_t kButtonMappingTargetSquare = 4;
constexpr uint8_t kButtonMappingTargetCross = 5;
constexpr uint8_t kButtonMappingTargetCircle = 6;
constexpr uint8_t kButtonMappingTargetTriangle = 7;
constexpr uint8_t kButtonMappingTargetL1 = 8;
constexpr uint8_t kButtonMappingTargetR1 = 9;
constexpr uint8_t kButtonMappingTargetL2 = 10;
constexpr uint8_t kButtonMappingTargetR2 = 11;
constexpr uint8_t kButtonMappingTargetCreate = 12;
constexpr uint8_t kButtonMappingTargetOptions = 13;
constexpr uint8_t kButtonMappingTargetL3 = 14;
constexpr uint8_t kButtonMappingTargetR3 = 15;
constexpr uint8_t kButtonMappingTargetPs = 16;
constexpr uint8_t kButtonMappingTargetTouchpad = 17;
constexpr uint8_t kButtonMappingTargetMute = 18;
constexpr uint8_t kButtonMappingTargetLeftFunction = 19;
constexpr uint8_t kButtonMappingTargetRightFunction = 20;
constexpr uint8_t kButtonMappingTargetLeftPaddle = 21;
constexpr uint8_t kButtonMappingTargetRightPaddle = 22;
constexpr uint8_t kButtonMappingTargetCount = 23;
constexpr uint8_t kButtonMappingTargetNone = 0xff;

constexpr uint8_t kDs5ButtonInputUp = 0;
constexpr uint8_t kDs5ButtonInputRight = 1;
constexpr uint8_t kDs5ButtonInputDown = 2;
constexpr uint8_t kDs5ButtonInputLeft = 3;
constexpr uint8_t kDs5ButtonInputSquare = 4;
constexpr uint8_t kDs5ButtonInputCross = 5;
constexpr uint8_t kDs5ButtonInputCircle = 6;
constexpr uint8_t kDs5ButtonInputTriangle = 7;
constexpr uint8_t kDs5ButtonInputL1 = 8;
constexpr uint8_t kDs5ButtonInputR1 = 9;
constexpr uint8_t kDs5ButtonInputL2 = 10;
constexpr uint8_t kDs5ButtonInputR2 = 11;
constexpr uint8_t kDs5ButtonInputCreate = 12;
constexpr uint8_t kDs5ButtonInputOptions = 13;
constexpr uint8_t kDs5ButtonInputL3 = 14;
constexpr uint8_t kDs5ButtonInputR3 = 15;
constexpr uint8_t kDs5ButtonInputPs = 16;
constexpr uint8_t kDs5ButtonInputTouchpad = 17;
constexpr uint8_t kDs5ButtonInputMute = 18;
constexpr uint8_t kDs5ButtonInputLeftFunction = 19;
constexpr uint8_t kDs5ButtonInputRightFunction = 20;
constexpr uint8_t kDs5ButtonInputLeftPaddle = 21;
constexpr uint8_t kDs5ButtonInputRightPaddle = 22;
constexpr uint8_t kDs5ButtonInputCount = 23;

constexpr uint8_t kNs2ProButtonInputUp = 0;
constexpr uint8_t kNs2ProButtonInputRight = 1;
constexpr uint8_t kNs2ProButtonInputDown = 2;
constexpr uint8_t kNs2ProButtonInputLeft = 3;
constexpr uint8_t kNs2ProButtonInputY = 4;
constexpr uint8_t kNs2ProButtonInputB = 5;
constexpr uint8_t kNs2ProButtonInputA = 6;
constexpr uint8_t kNs2ProButtonInputX = 7;
constexpr uint8_t kNs2ProButtonInputL = 8;
constexpr uint8_t kNs2ProButtonInputR = 9;
constexpr uint8_t kNs2ProButtonInputZL = 10;
constexpr uint8_t kNs2ProButtonInputZR = 11;
constexpr uint8_t kNs2ProButtonInputMinus = 12;
constexpr uint8_t kNs2ProButtonInputPlus = 13;
constexpr uint8_t kNs2ProButtonInputL3 = 14;
constexpr uint8_t kNs2ProButtonInputR3 = 15;
constexpr uint8_t kNs2ProButtonInputHome = 16;
constexpr uint8_t kNs2ProButtonInputCapture = 17;
constexpr uint8_t kNs2ProButtonInputGL = 18;
constexpr uint8_t kNs2ProButtonInputGR = 19;
constexpr uint8_t kNs2ProButtonInputCount = 20;

void button_mapping_default();
void button_mapping_load();
bool button_mapping_save();

size_t button_mapping_get_ds5(uint8_t *buffer, size_t reqlen);
size_t button_mapping_get_ns2pro(uint8_t *buffer, size_t reqlen);
void button_mapping_set_ds5(const uint8_t *buffer, size_t len);
void button_mapping_set_ns2pro(const uint8_t *buffer, size_t len);

void button_mapping_apply_ds5(uint8_t *report63, size_t len);
uint32_t button_mapping_apply_ns2pro(uint32_t physical_buttons);

#endif
