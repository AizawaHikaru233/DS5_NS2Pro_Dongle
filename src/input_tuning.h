#ifndef DS5_BRIDGE_INPUT_TUNING_H
#define DS5_BRIDGE_INPUT_TUNING_H

// NS2Pro gyro game-side speed tuning lives in the DS5 0x05 calibration
// feature report only. Keep raw input reports untouched.
constexpr float kNs2ProDs5CalibrationGyroYGameScale = 15.0f;
constexpr float kNs2ProDs5CalibrationGyroZGameScale = 15.0f;
constexpr float kNs2ProBleDs5CalibrationGyroYGameScale = 115.0f;
constexpr float kNs2ProBleDs5CalibrationGyroZGameScale = 115.0f;

#endif
