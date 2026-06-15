#ifndef DS5_BRIDGE_NS2PRO_RUMBLE_H
#define DS5_BRIDGE_NS2PRO_RUMBLE_H

#include <cstddef>
#include <cstdint>

struct Ns2ProRumbleDebug {
    uint8_t last_sequence;
    uint8_t last_mixed_low;
    uint8_t last_mixed_high;
    uint8_t last_source;
    uint8_t input_count;
    uint8_t build_count;
    uint8_t send_attempt_count;
    uint8_t send_success_count;
    uint8_t send_failure_count;
    uint8_t last_send_stage;
    uint8_t ble_sent_count;
    uint8_t usb_queued_count;
    uint8_t last_report_head[12];
};

void ns2pro_rumble_init();
void ns2pro_rumble_on_ds5_output_report(const uint8_t *report, size_t len);
void ns2pro_rumble_on_haptics_audio(const int8_t *samples, size_t len);
void ns2pro_rumble_set_direct(uint8_t low, uint8_t high);
void ns2pro_rumble_set_direct_quad(uint8_t left_high, uint8_t left_low, uint8_t right_high, uint8_t right_low);
bool ns2pro_rumble_build_output_report(uint8_t out_report[64]);
bool ns2pro_rumble_should_stream_output();
void ns2pro_rumble_note_input_observed();
void ns2pro_rumble_note_report_built();
void ns2pro_rumble_note_output_attempt(bool success, uint8_t stage);
void ns2pro_rumble_note_output_sent(bool usb_queued);
void ns2pro_rumble_get_debug(Ns2ProRumbleDebug *out_debug);

#endif
