#include "ns2pro_serial_bridge.h"

#include <cstring>

#include "pico/time.h"
#include "tusb.h"

#if !defined(ENABLE_NS2PRO_SERIAL_BRIDGE)
#define ENABLE_NS2PRO_SERIAL_BRIDGE 0
#endif

namespace {

constexpr uint8_t SERIAL_MAGIC0 = 0xE5;
constexpr uint8_t SERIAL_MAGIC1 = 0x50;
constexpr uint8_t SERIAL_FRAME_NS2PRO_RAW = 0x03;
constexpr uint8_t SERIAL_FRAME_NS2PRO_OUTPUT = 0x04;
constexpr size_t SERIAL_BUFFER_SIZE = 512;
constexpr size_t SERIAL_MAX_PAYLOAD = 255;
constexpr size_t NS2PRO_MAX_INPUT_PAYLOAD = 64;
constexpr size_t NS2PRO_OUTPUT_REPORT_LEN = 64;
constexpr size_t NS2PRO_OUTPUT_FRAME_LEN = 5 + NS2PRO_OUTPUT_REPORT_LEN + 1;

uint8_t rx_buffer[SERIAL_BUFFER_SIZE];
size_t rx_len = 0;
uint8_t latest_payload[NS2PRO_MAX_INPUT_PAYLOAD];
size_t latest_payload_len = 0;
bool latest_payload_ready = false;
bool saw_input_activity = false;
uint32_t last_input_ms = 0;
uint8_t tx_sequence = 0;
uint8_t last_output_status = 0;
uint8_t pending_output_payload[NS2PRO_OUTPUT_REPORT_LEN];
bool pending_output_ready = false;

uint8_t frame_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

void compact_rx(size_t consumed) {
    if (consumed == 0) {
        return;
    }
    if (consumed >= rx_len) {
        rx_len = 0;
        return;
    }
    memmove(rx_buffer, rx_buffer + consumed, rx_len - consumed);
    rx_len -= consumed;
}

void flush_output_queue() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    if (!pending_output_ready) {
        return;
    }

    if (tud_cdc_write_available() < NS2PRO_OUTPUT_FRAME_LEN) {
        return;
    }

    uint8_t frame[NS2PRO_OUTPUT_FRAME_LEN];
    frame[0] = SERIAL_MAGIC0;
    frame[1] = SERIAL_MAGIC1;
    frame[2] = SERIAL_FRAME_NS2PRO_OUTPUT;
    frame[3] = static_cast<uint8_t>(NS2PRO_OUTPUT_REPORT_LEN);
    frame[4] = tx_sequence++;
    memcpy(frame + 5, pending_output_payload, NS2PRO_OUTPUT_REPORT_LEN);
    frame[5 + NS2PRO_OUTPUT_REPORT_LEN] = frame_checksum(frame, 5 + NS2PRO_OUTPUT_REPORT_LEN);

    const uint32_t written = tud_cdc_write(frame, sizeof(frame));
    tud_cdc_write_flush();
    if (written != sizeof(frame)) {
        last_output_status = 4;
        return;
    }

    pending_output_ready = false;
    last_output_status = 3;
#endif
}

void parse_frames() {
    size_t cursor = 0;
    while (rx_len - cursor >= 6) {
        if (rx_buffer[cursor] != SERIAL_MAGIC0 || rx_buffer[cursor + 1] != SERIAL_MAGIC1) {
            ++cursor;
            continue;
        }

        const size_t payload_len = rx_buffer[cursor + 3];
        if (payload_len > SERIAL_MAX_PAYLOAD) {
            ++cursor;
            continue;
        }

        const size_t frame_len = 5 + payload_len + 1;
        if (rx_len - cursor < frame_len) {
            break;
        }

        const uint8_t *frame = rx_buffer + cursor;
        if (frame_checksum(frame, frame_len - 1) != frame[frame_len - 1]) {
            ++cursor;
            continue;
        }

        if (frame[2] == SERIAL_FRAME_NS2PRO_RAW && payload_len <= NS2PRO_MAX_INPUT_PAYLOAD) {
            memcpy(latest_payload, frame + 5, payload_len);
            latest_payload_len = payload_len;
            latest_payload_ready = true;
            saw_input_activity = true;
            last_input_ms = to_ms_since_boot(get_absolute_time());
        }

        cursor += frame_len;
    }

    compact_rx(cursor);
    if (rx_len > 256) {
        compact_rx(rx_len - 128);
    }
}

} // namespace

void ns2pro_serial_bridge_init() {
    rx_len = 0;
    latest_payload_len = 0;
    latest_payload_ready = false;
    saw_input_activity = false;
    last_input_ms = 0;
    tx_sequence = 0;
    last_output_status = 0;
    pending_output_ready = false;
}

bool ns2pro_serial_bridge_enabled() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    return true;
#else
    return false;
#endif
}

bool ns2pro_serial_bridge_has_activity() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    return saw_input_activity;
#else
    return false;
#endif
}

uint32_t ns2pro_serial_bridge_last_input_ms() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    return last_input_ms;
#else
    return 0;
#endif
}

void ns2pro_serial_bridge_task() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    while (tud_cdc_available() > 0) {
        if (rx_len >= sizeof(rx_buffer)) {
            rx_len = 0;
        }
        const uint32_t read = tud_cdc_read(rx_buffer + rx_len, sizeof(rx_buffer) - rx_len);
        if (read == 0) {
            break;
        }
        rx_len += read;
        parse_frames();
    }
    flush_output_queue();
#endif
}

bool ns2pro_serial_bridge_poll_latest_input(uint8_t *payload, size_t *len) {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    if (!latest_payload_ready || !payload || !len) {
        return false;
    }

    memcpy(payload, latest_payload, latest_payload_len);
    *len = latest_payload_len;
    latest_payload_ready = false;
    return true;
#else
    (void) payload;
    (void) len;
    return false;
#endif
}

bool ns2pro_serial_bridge_send_output_report(const uint8_t *payload, size_t len) {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    if (!payload || len != NS2PRO_OUTPUT_REPORT_LEN) {
        last_output_status = 1;
        return false;
    }

    uint8_t frame[5 + NS2PRO_OUTPUT_REPORT_LEN + 1];
    frame[0] = SERIAL_MAGIC0;
    frame[1] = SERIAL_MAGIC1;
    frame[2] = SERIAL_FRAME_NS2PRO_OUTPUT;
    frame[3] = static_cast<uint8_t>(len);
    frame[4] = tx_sequence++;
    memcpy(frame + 5, payload, len);
    frame[5 + len] = frame_checksum(frame, 5 + len);
    if (!pending_output_ready && tud_cdc_write_available() >= sizeof(frame)) {
        const uint32_t written = tud_cdc_write(frame, sizeof(frame));
        tud_cdc_write_flush();
        last_output_status = written == sizeof(frame) ? 3 : 4;
        return written == sizeof(frame);
    }

    memcpy(pending_output_payload, payload, NS2PRO_OUTPUT_REPORT_LEN);
    pending_output_ready = true;
    flush_output_queue();
    last_output_status = pending_output_ready ? 2 : 3;
    return true;
#else
    (void) payload;
    (void) len;
    last_output_status = 5;
    return false;
#endif
}

uint8_t ns2pro_serial_bridge_last_output_status() {
#if ENABLE_NS2PRO_SERIAL_BRIDGE
    return last_output_status;
#else
    return 5;
#endif
}
