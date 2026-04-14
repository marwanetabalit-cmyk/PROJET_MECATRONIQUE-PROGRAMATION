#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "motors.h"
#include "encoders.h"
#include "ultrasound.h"
#include "servo_motor.h"

// --- Constants moteurs / ultrason ---
#define DISTANCE_STOP 5.0f
#define TEMPS_ARRET 5000

// --- Constants servo ---
#define SERVO_GPIO 27
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_CENTER 90
#define SERVO_DELTA 5   // amplitude réduite ±5°

uint servo_slice;
int servo_angle = SERVO_CENTER - SERVO_DELTA;
int servo_step = SERVO_DELTA;

void servo_init(void)
{
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(SERVO_GPIO);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 20000);  // 50 Hz

    pwm_init(servo_slice, &config, true);
}

void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse =
        SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;

    pwm_set_gpio_level(SERVO_GPIO, pulse);
}

int main()
{
    stdio_init_all();
    sleep_ms(500);
    printf("Test moteurs PWM + ultrason + servo FT90M\n");

    motor_init();
    encoders_init();
    ultrason_init();
    servo_init();

    // --- Timer indépendant pour le servo ---
    uint32_t last_servo_time = to_ms_since_boot(get_absolute_time());

    while (true)
    {
        // --- Servo : bouge toutes les 2 secondes ---
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_servo_time >= 2000)
        {
            servo_set_angle(servo_angle);
            servo_angle += servo_step;

            if (servo_angle >= SERVO_CENTER + SERVO_DELTA ||
                servo_angle <= SERVO_CENTER - SERVO_DELTA)
            {
                servo_step = -servo_step;
            }

            last_servo_time = now;
        }

        // --- Moteurs PWM + Ultrason ---
        for (int speed = 0; speed <= 100; speed += 10)
        {
            motor_set_speed(slice_A, speed);
            motor_set_speed(slice_B, speed);

            float distance = lireDistance();
            if (distance > 0)
                printf("Distance: %.1f cm\n", distance);
            else
                printf("Distance: --\n");

            if (distance > 0 && distance < DISTANCE_STOP)
            {
                printf("Obstacle < %.1f cm détecté ! Arrêt 5s\n", DISTANCE_STOP);
                motor_set_speed(slice_A, 0);
                motor_set_speed(slice_B, 0);
                sleep_ms(TEMPS_ARRET);
            }

            sleep_ms(500);
        }

        for (int speed = 100; speed >= 0; speed -= 10)
        {
            motor_set_speed(slice_A, speed);
            motor_set_speed(slice_B, speed);

            float distance = lireDistance();
            if (distance > 0)
                printf("Distance: %.1f cm\n", distance);
            else
                printf("Distance: --\n");

            if (distance > 0 && distance < DISTANCE_STOP)
            {
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

// test GIT
