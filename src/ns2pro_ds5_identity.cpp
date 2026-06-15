#include "ns2pro_ds5_identity.h"

#include <cmath>
#include <cstddef>

#include "input_tuning.h"

using std::vector;

namespace {

static int16_t read_le16(const vector<uint8_t> &data, size_t offset) {
    return static_cast<int16_t>(data[offset] | (data[offset + 1] << 8));
}

static void write_le16(vector<uint8_t> &data, size_t offset, int16_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xff);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

static int16_t tune_calibration_endpoint(int16_t bias, int16_t endpoint, float scale) {
    if (scale <= 0.0f) {
        return endpoint;
    }
    const float tuned = static_cast<float>(bias) +
        (static_cast<float>(endpoint) - static_cast<float>(bias)) / scale;
    const long rounded = lroundf(tuned);
    if (rounded < -32768L) {
        return -32768;
    }
    if (rounded > 32767L) {
        return 32767;
    }
    return static_cast<int16_t>(rounded);
}

static void tune_gyro_calibration_group(
    vector<uint8_t> &report,
    size_t report_id_bytes,
    size_t bias_offset,
    size_t plus_offset,
    size_t minus_offset,
    float scale
) {
    if (report.size() < report_id_bytes + minus_offset + 2) {
        return;
    }

    const size_t bias_index = report_id_bytes + bias_offset;
    const size_t plus_index = report_id_bytes + plus_offset;
    const size_t minus_index = report_id_bytes + minus_offset;

    const int16_t bias = read_le16(report, bias_index);
    const int16_t plus = read_le16(report, plus_index);
    const int16_t minus = read_le16(report, minus_index);

    write_le16(report, plus_index, tune_calibration_endpoint(bias, plus, scale));
    write_le16(report, minus_index, tune_calibration_endpoint(bias, minus, scale));
}

static void apply_ns2pro_gyro_game_tuning(
    vector<uint8_t> &report,
    float gyro_y_scale,
    float gyro_z_scale
) {
    constexpr size_t kReportIdBytes = 1;
    tune_gyro_calibration_group(
        report,
        kReportIdBytes,
        2,
        10,
        12,
        gyro_y_scale
    );
    tune_gyro_calibration_group(
        report,
        kReportIdBytes,
        4,
        14,
        16,
        gyro_z_scale
    );
}

size_t local_feature_payload_size(uint8_t report_id) {
    switch (report_id) {
        case 0x05: return 40;
        case 0x08: return 47;
        case 0x09: return 19;
        case 0x0a: return 26;
        case 0x0b: return 41;
        case 0x0c: return 41;
        case 0x20: return 63;
        case 0x21: return 4;
        case 0x22: return 63;
        case 0x80: return 63;
        case 0x81: return 63;
        case 0x82: return 9;
        default: return 0;
    }
}

vector<uint8_t> build_wired_feature_report(uint8_t report_id, size_t payload_size, bool ns2pro_tuned) {
    switch (report_id) {
        case 0x05:
            {
                vector<uint8_t> report{
                0x05, 0xfa, 0xff, 0x08, 0x00, 0xf0, 0xff, 0x98, 0x22, 0x64,
                0xdd, 0x91, 0x22, 0x84, 0xdd, 0x96, 0x22, 0x48, 0xdd, 0x1c,
                0x02, 0x1c, 0x02, 0xe9, 0x1f, 0xd4, 0xdf, 0x98, 0x20, 0xad,
                0xe0, 0x0e, 0x20, 0xf8, 0xdf, 0x0e, 0x00, 0x9e, 0xe6, 0x02,
                0x90
                };
                if (ns2pro_tuned) {
                    apply_ns2pro_gyro_game_tuning(
                        report,
                        kNs2ProDs5CalibrationGyroYGameScale,
                        kNs2ProDs5CalibrationGyroZGameScale
                    );
                }
                return report;
            }
        case 0x09:
            return vector<uint8_t>{
                0x09, 0x04, 0x53, 0x11, 0x48, 0x46, 0x44, 0x08, 0x25, 0x00,
                0x4a, 0x5b, 0x0f, 0x72, 0x36, 0x52, 0xc8, 0x08, 0x44, 0x1c
            };
        case 0x20:
            return vector<uint8_t>{
                0x20, 0x53, 0x65, 0x70, 0x20, 0x31, 0x39, 0x20, 0x32, 0x30,
                0x32, 0x35, 0x31, 0x30, 0x3a, 0x31, 0x34, 0x3a, 0x33, 0x30,
                0x02, 0x00, 0x44, 0x00, 0x16, 0x02, 0x00, 0x01, 0x8b, 0x00,
                0x00, 0x01, 0xc1, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x17, 0x02, 0x00, 0x00, 0x14, 0x00,
                0x00, 0x00, 0x0a, 0x00, 0x02, 0x00, 0x06, 0x00, 0x00, 0x00,
                0xee, 0xee, 0x8a, 0x96
            };
        case 0x22:
            return vector<uint8_t>{
                0x22, 0x02, 0x00, 0x16, 0x02, 0x00, 0x01, 0x8b, 0x00, 0x00,
                0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x53, 0x11,
                0x48, 0x46, 0x44, 0x0a, 0x00, 0x02, 0x00, 0x06, 0x00, 0x00,
                0x00, 0x19, 0x00, 0x00, 0x00, 0xa6, 0x0a, 0x11, 0xaa, 0x01,
                0x03, 0x05, 0x03, 0x00, 0x00, 0x99, 0xbf, 0xfe, 0x28, 0x05,
                0x9e, 0x82, 0x72, 0x14, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
                0xae, 0xa8, 0xed, 0x81
            };
        case 0x80:
            return vector<uint8_t>(payload_size + 1, 0);
        case 0x81: {
            vector<uint8_t> report(payload_size + 1, 0);
            report[0] = 0x81;
            report[1] = 0x01;
            return report;
        }
        case 0x82: {
            vector<uint8_t> report(payload_size + 1, 0);
            report[0] = 0x82;
            report[1] = 0x01;
            return report;
        }
        default: {
            vector<uint8_t> report(payload_size + 1, 0);
            report[0] = report_id;
            return report;
        }
    }
}

vector<uint8_t> build_ble_feature_report(uint8_t report_id, size_t payload_size, bool ns2pro_tuned) {
    switch (report_id) {
        case 0x05:
            {
                vector<uint8_t> report = build_wired_feature_report(report_id, payload_size, false);
                if (ns2pro_tuned) {
                    apply_ns2pro_gyro_game_tuning(
                        report,
                        kNs2ProBleDs5CalibrationGyroYGameScale,
                        kNs2ProBleDs5CalibrationGyroZGameScale
                    );
                }
                return report;
            }
        default:
            return build_wired_feature_report(report_id, payload_size, ns2pro_tuned);
    }
}

} // namespace

vector<uint8_t> ns2pro_build_local_feature_report(
    uint8_t report_id,
    bool ns2pro_tuned,
    Ns2ProIdentityProfile profile
) {
    const size_t payload_size = local_feature_payload_size(report_id);
    if (payload_size == 0) {
        return {};
    }

    switch (profile) {
        case Ns2ProIdentityProfile::Ble:
            return build_ble_feature_report(report_id, payload_size, ns2pro_tuned);
        case Ns2ProIdentityProfile::Wired:
        default:
            return build_wired_feature_report(report_id, payload_size, ns2pro_tuned);
    }
}

