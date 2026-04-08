#include "hal_servo.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

void servo_init(void) {
    printf("[SERVO] Initialisation sur GPIO %d\n", SERVO_PIN);
    
    // Configure la pin en PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    
    // Configuration pour servomoteur standard (50Hz)
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    
    // 50Hz = 20ms période
    // Clock diviseur: 125MHz / 64 = 1.953125MHz
    // Wrap: 1.953125MHz / 50Hz = 39062.5
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_config_set_wrap(&config, 39062);
    
    pwm_init(slice_num, &config, true);
    
    // Position initiale au centre (90°)
    servo_set_angle(90);
}

void servo_set_angle(uint angle) {
    if(angle > 180) angle = 180;
    
    // Conversion angle -> largeur d'impulsion
    // Servo standard: 0° = 1000µs, 180° = 2000µs
    // Pour wrap=39062 (20ms): 1000µs = 1953, 2000µs = 3906
    uint pulse_us = 1000 + (angle * 1000 / 180);
    uint level = (pulse_us * 39062) / 20000;
    
    pwm_set_gpio_level(SERVO_PIN, level);
}

void servo_celebrate(void) {
    printf("[SERVO] Animation de célébration!\n");
    
    // Animation: 3 cycles gauche-droite
    for(int i = 0; i < 3; i++) {
        servo_set_angle(0);     // Regarde à gauche
        sleep_ms(250);
        servo_set_angle(180);   // Regarde à droite
        sleep_ms(250);
    }
    
    // Retour au centre
    servo_set_angle(90);
    printf("[SERVO] Animation terminée\n");
}