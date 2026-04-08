#include "hal_ultrasound.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"

// Initialise les pins
void ultrasound_init() {
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_put(TRIG_PIN, 0);
}

// Mesure distance
float ultrasound_measure() {
    // Envoi impulsion de 10µs
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
    
    // Attend front montant
    uint32_t start = time_us_32();
    while(gpio_get(ECHO_PIN) == 0) {
        if(time_us_32() - start > 30000) return -1; // Timeout
    }
    
    // Mesure durée impulsion
    start = time_us_32();
    while(gpio_get(ECHO_PIN) == 1) {
        if(time_us_32() - start > 30000) return -1; // Timeout
    }
    
    // Calcul distance
    uint32_t duration = time_us_32() - start;
    return (duration * 0.0343) / 2.0f; // Vitesse du son: 343 m/s
}

// Détection obstacle
bool obstacle_detected(float threshold_cm) {
    float dist = ultrasound_measure();
    return (dist > 0 && dist < threshold_cm);
}