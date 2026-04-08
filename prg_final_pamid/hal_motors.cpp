#include "hal_motors.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>  // Changé de <cstdlib> à <stdlib.h>

int ajout_calibration = 50; // Variable pour ajuster la calibration des moteurs
// Variables PWM
static uint slice_a, slice_b;
static uint channel_a1, channel_a2, channel_b1, channel_b2;

// Fonction d'initialisation simplifiée
void motors_init() {
    printf("[MOTORS] Initialisation...\n");
    
    // Configure les pins comme sorties GPIO normales d'abord
    gpio_init(MOTOR_A_IN1);
    gpio_init(MOTOR_A_IN2);
    gpio_init(MOTOR_B_IN1);
    gpio_init(MOTOR_B_IN2);
    
    gpio_set_dir(MOTOR_A_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_A_IN2, GPIO_OUT);
    gpio_set_dir(MOTOR_B_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_B_IN2, GPIO_OUT);
    
    // Initialement à 0
    gpio_put(MOTOR_A_IN1, 0);
    gpio_put(MOTOR_A_IN2, 0);
    gpio_put(MOTOR_B_IN1, 0);
    gpio_put(MOTOR_B_IN2, 0);
    
    // Configure seulement IN1 en PWM, IN2 reste en GPIO pour la direction
    gpio_set_function(MOTOR_A_IN1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_B_IN1, GPIO_FUNC_PWM);
    
    slice_a = pwm_gpio_to_slice_num(MOTOR_A_IN1);
    slice_b = pwm_gpio_to_slice_num(MOTOR_B_IN1);
    
    channel_a1 = pwm_gpio_to_channel(MOTOR_A_IN1);
    channel_b1 = pwm_gpio_to_channel(MOTOR_B_IN1);
    
    // Configure PWM
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_config_set_wrap(&config, 1562);
    
    pwm_init(slice_a, &config, true);
    pwm_init(slice_b, &config, true);
    
    printf("[MOTORS] Initialisé - PWM sur IN1, direction sur IN2\n");
}

// Contrôle simplifié : PWM sur IN1, direction sur IN2
static void motor_control_simple(uint pwm_pin, uint dir_pin, uint slice, uint channel, int speed) {
    if(speed > 100) speed = 100;
    if(speed < -100) speed = -100;
    
    uint16_t pwm_level = (abs(speed) * 1562) / 100;
    
    if(speed > 0) {
        // Avant : IN1 = PWM, IN2 = 0
        gpio_put(dir_pin, 0);
        pwm_set_chan_level(slice, channel, pwm_level);
    }
    else if(speed < 0) {
        // Arrière : IN1 = PWM, IN2 = 1
        gpio_put(dir_pin, 1);
        pwm_set_chan_level(slice, channel, pwm_level);
    }
    else {
        // Stop : IN1 = 0, IN2 = 0
        gpio_put(dir_pin, 0);
        pwm_set_chan_level(slice, channel, 0);
    }
}

// Contrôle individuel - ESSAYEZ CES COMBINAISONS
void motor_left_set(int speed) {
    // Essayer d'abord sans inversion
    motor_control_simple(MOTOR_A_IN1, MOTOR_A_IN2, slice_a, channel_a1, speed);
}

void motor_right_set(int speed) {
    // Si le robot tourne au lieu d'avancer, inversez le signe :
    // motor_control_simple(MOTOR_B_IN1, MOTOR_B_IN2, slice_b, channel_b1, -speed);
    motor_control_simple(MOTOR_B_IN1, MOTOR_B_IN2, slice_b, channel_b1, speed);
}

// ... reste identique ...

// Commandes prédéfinies
void motors_forward(uint8_t speed) {
    motor_left_set(-(speed+ajout_calibration)); // Ajustement calibration
    motor_right_set(speed);
}

void motors_backward(uint8_t speed) {
    motor_left_set(-(speed+ajout_calibration));
    motor_right_set(-speed);
}

void motors_rotate_right(uint8_t speed) {
    motor_left_set((speed+ajout_calibration));    // Gauche avance
    motor_right_set(-speed);  // Droit recule
}

void motors_rotate_left(uint8_t speed) {
    motor_left_set(-(speed+ajout_calibration));   // Gauche recule
    motor_right_set(speed);   // Droit avance
}

void motors_stop() {
    motor_left_set(0);
    motor_right_set(0);
}