#include "ultrasound.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Broches du capteur ultrason
#define PIN_TRIG 6
#define PIN_ECHO 7

// Vitesse du son divisée par 2 pour le calcul de distance (aller-retour)
#define VITESSE_SON 0.034f / 2.0f

// Initialisation du capteur ultrason
void ultrason_init(void) {
    gpio_init(PIN_TRIG);           // Initialisation broche Trigger
    gpio_set_dir(PIN_TRIG, GPIO_OUT);
    gpio_put(PIN_TRIG, 0);         // Valeur initiale LOW

    gpio_init(PIN_ECHO);           // Initialisation broche Echo
    gpio_set_dir(PIN_ECHO, GPIO_IN);

    stdio_init_all();              // Initialisation pour printf (optionnel)
    sleep_ms(500);                 // Pause pour stabiliser
}

// Lit la distance en cm depuis le capteur ultrason
float lireDistance(void) {
    // Envoi d'une impulsion de 10 µs sur la broche Trigger
    gpio_put(PIN_TRIG, 1);
    sleep_us(10);
    gpio_put(PIN_TRIG, 0);

    // Attente du front montant sur Echo
    uint32_t start = time_us_32();
    while (!gpio_get(PIN_ECHO)) {
        if (time_us_32() - start > 30000) return -1.0f; // Timeout si rien détecté
    }

    // Mesure de la durée de l'impulsion Echo
    uint32_t echo_start = time_us_32();
    while (gpio_get(PIN_ECHO)) {
        if (time_us_32() - echo_start > 30000) return -1.0f; // Timeout si trop long
    }

    uint32_t echo_end = time_us_32();

    // Calcul de la distance en cm
    return (echo_end - echo_start) * VITESSE_SON;
}
