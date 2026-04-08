#include <stdio.h>
#include "pico/stdlib.h"
#include "Moteurs.h"
#include "Codeur.h"
#include "Ultrason.h"

#define DISTANCE_STOP 5.0f    // cm
#define TEMPS_ARRET 5000      // ms

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("Test moteurs PWM et ultrason avec stop si obstacle\n");

    motor_init();
    encoders_init();
    ultrason_init();

    while (true) {
        for (int speed = 0; speed <= 100; speed += 10) {
            motor_set_speed(slice_A, speed);
            motor_set_speed(slice_B, speed);

            float distance = lireDistance();
            if (distance > 0)
                printf("Distance: %.1f cm\n", distance);
            else
                printf("Distance: --\n");

            if (distance > 0 && distance < DISTANCE_STOP) {
                printf("Obstacle < %.1f cm détecté ! Arrêt 5s\n", DISTANCE_STOP);
                motor_set_speed(slice_A, 0);
                motor_set_speed(slice_B, 0);
                sleep_ms(TEMPS_ARRET);
            }

            sleep_ms(500);
        }

        for (int speed = 100; speed >= 0; speed -= 10) {
            motor_set_speed(slice_A, speed);
            motor_set_speed(slice_B, speed);

            float distance = lireDistance();
            if (distance > 0)
                printf("Distance: %.1f cm\n", distance);
            else
                printf("Distance: --\n");

            if (distance > 0 && distance < DISTANCE_STOP) {
                printf("Obstacle < %.1f cm détecté ! Arrêt 5s\n", DISTANCE_STOP);
                motor_set_speed(slice_A, 0);
                motor_set_speed(slice_B, 0);
                sleep_ms(TEMPS_ARRET);
            }

            sleep_ms(500);
        }

        motor_set_speed(slice_A, 0);
        motor_set_speed(slice_B, 0);
        sleep_ms(1000);
    }
}
