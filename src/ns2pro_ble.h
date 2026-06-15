#ifndef DS5_BRIDGE_NS2PRO_BLE_H
#define DS5_BRIDGE_NS2PRO_BLE_H

#include <cstddef>
#include <cstdint>

void ns2pro_ble_init();
void ns2pro_ble_stack_init();
void ns2pro_ble_task();
void ns2pro_ble_request_pairing();
void ns2pro_ble_clear_bond();
void ns2pro_ble_set_pairing_service_enabled(bool enabled);
bool ns2pro_ble_pairing_service_enabled();
void ns2pro_ble_set_idle_pairing_enabled(bool enabled);
bool ns2pro_ble_idle_pairing_enabled();
bool ns2pro_ble_poll_latest_input(uint8_t *payload, size_t *len);
bool ns2pro_ble_connected();
bool ns2pro_ble_send_output_report(const uint8_t *report, size_t len);
uint8_t ns2pro_ble_last_output_status();
void ns2pro_ble_on_classic_pair_success();
void ns2pro_ble_on_classic_disconnect();

#endif
