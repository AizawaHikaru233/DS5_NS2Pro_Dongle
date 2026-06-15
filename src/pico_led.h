#ifndef DS5_NS2PRO_DONGLE_PICO_LED_H
#define DS5_NS2PRO_DONGLE_PICO_LED_H

void pico_led_set(bool on);
void pico_led_set_override(bool on);
void pico_led_restore_configured_state();

#endif
