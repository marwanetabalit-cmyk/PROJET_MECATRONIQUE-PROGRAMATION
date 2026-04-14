#include "ultrasound.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"

// --- CONFIGURATION DES BROCHES ---
#define PIN_TRIG 10
#define PIN_ECHO 11

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile float distance_cm = 999.0; // Initialisé à une valeur très grande (pas d'obstacle)

// Routine d'interruption (ISR) appelée automatiquement quand la broche ECHO change d'état
void echo_pin_isr(uint gpio, uint32_t events) {
    if (gpio == PIN_ECHO) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            // Le signal monte : le son vient de partir
            start_time = time_us_32();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            // Le signal descend : le son est revenu
            end_time = time_us_32();
            uint32_t delta = end_time - start_time;
            // Vitesse du son = ~340 m/s, soit 29 microsecondes par cm (aller-retour)
            distance_cm = (float)delta / 58.0;
        }
    }
}

void ultrasound_init(void) {
    gpio_init(PIN_TRIG);
    gpio_set_dir(PIN_TRIG, GPIO_OUT);
    gpio_put(PIN_TRIG, 0);

    gpio_init(PIN_ECHO);
    gpio_set_dir(PIN_ECHO, GPIO_IN);

    // Configuration de l'interruption sur la broche ECHO
    gpio_set_irq_enabled_with_callback(PIN_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_pin_isr);
}

void ultrasound_trigger(void) {
    // Génère une impulsion de 10 microsecondes sans bloquer longtemps le CPU
    gpio_put(PIN_TRIG, 1);
    sleep_us(10); 
    gpio_put(PIN_TRIG, 0);
}

float ultrasound_get_distance_cm(void) {
    return distance_cm;
}
    uint32_t echo_end = time_us_32();
    return (echo_end - echo_start) * VITESSE_SON;
}
