#include "pico_led.h"

#include "config.h"
#include "pico/cyw43_arch.h"

void pico_led_set(bool on) {
    if (get_config().disable_pico_led) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        return;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

void pico_led_set_override(bool on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

void pico_led_restore_configured_state() {
    pico_led_set(true);
}
