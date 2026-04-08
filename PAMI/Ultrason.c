#include "Ultrason.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define PIN_TRIG 6
#define PIN_ECHO 7
#define VITESSE_SON 0.034f / 2.0f

void ultrason_init(void) {
    gpio_init(PIN_TRIG);
    gpio_set_dir(PIN_TRIG, GPIO_OUT);
    gpio_put(PIN_TRIG, 0);

    gpio_init(PIN_ECHO);
    gpio_set_dir(PIN_ECHO, GPIO_IN);

    stdio_init_all();
    sleep_ms(500);
}

float lireDistance(void) {
    gpio_put(PIN_TRIG, 1);
    sleep_us(10);
    gpio_put(PIN_TRIG, 0);

    uint32_t start = time_us_32();
    while (!gpio_get(PIN_ECHO)) {
        if (time_us_32() - start > 30000) return -1.0f;
    }

    uint32_t echo_start = time_us_32();
    while (gpio_get(PIN_ECHO)) {
        if (time_us_32() - echo_start > 30000) return -1.0f;
    }

    uint32_t echo_end = time_us_32();
    return (echo_end - echo_start) * VITESSE_SON;
}
