#ifndef DS5_BRIDGE_NS2PRO_SERIAL_BRIDGE_H
#define DS5_BRIDGE_NS2PRO_SERIAL_BRIDGE_H

#include <cstddef>
#include <cstdint>

void ns2pro_serial_bridge_init();
void ns2pro_serial_bridge_task();
bool ns2pro_serial_bridge_poll_latest_input(uint8_t *payload, size_t *len);
bool ns2pro_serial_bridge_send_output_report(const uint8_t *payload, size_t len);
bool ns2pro_serial_bridge_enabled();
bool ns2pro_serial_bridge_has_activity();
uint32_t ns2pro_serial_bridge_last_input_ms();
uint8_t ns2pro_serial_bridge_last_output_status();

#endif
