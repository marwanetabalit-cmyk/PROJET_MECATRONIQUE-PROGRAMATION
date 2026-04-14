#include "motors.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- CONFIGURATION DES BROCHES (À CHANGER SELON TON CÂBLAGE) ---
// Moteur Gauche (Left)
#define PIN_MOTOR_L_PWM 2
#define PIN_MOTOR_L_DIR 3 

// Moteur Droit (Right)
#define PIN_MOTOR_R_PWM 4
#define PIN_MOTOR_R_DIR 5

// Constante PWM (Pour une fréquence de ~10kHz)
#define PWM_WRAP 12500 

void motors_init(void) {
    // Configuration des broches de direction en sortie
    gpio_init(PIN_MOTOR_L_DIR);
    gpio_set_dir(PIN_MOTOR_L_DIR, GPIO_OUT);
    gpio_init(PIN_MOTOR_R_DIR);
    gpio_set_dir(PIN_MOTOR_R_DIR, GPIO_OUT);

    // Configuration des broches PWM
    gpio_set_function(PIN_MOTOR_L_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_R_PWM, GPIO_FUNC_PWM);

    // Récupération des "slices" (blocs matériels PWM du RP2040)
    uint slice_L = pwm_gpio_to_slice_num(PIN_MOTOR_L_PWM);
    uint slice_R = pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM);

    // Configuration du PWM (Fréquence = Clock / (Wrap * ClkDiv))
    pwm_set_wrap(slice_L, PWM_WRAP);
    pwm_set_wrap(slice_R, PWM_WRAP);
    
    // Démarrage des signaux PWM à 0%
    pwm_set_gpio_level(PIN_MOTOR_L_PWM, 0);
    pwm_set_gpio_level(PIN_MOTOR_R_PWM, 0);
    
    pwm_set_enabled(slice_L, true);
    pwm_set_enabled(slice_R, true);
}

void motors_set_speed(int speed_left, int speed_right) {
    // Saturation de sécurité
    if (speed_left > 100) speed_left = 100;
    if (speed_left < -100) speed_left = -100;
    if (speed_right > 100) speed_right = 100;
    if (speed_right < -100) speed_right = -100;

    // --- MOTEUR GAUCHE ---
    if (speed_left >= 0) {
        gpio_put(PIN_MOTOR_L_DIR, 1); // Marche avant (inverse si ton robot recule)
        pwm_set_gpio_level(PIN_MOTOR_L_PWM, (speed_left * PWM_WRAP) / 100);
    } else {
        gpio_put(PIN_MOTOR_L_DIR, 0); // Marche arrière
        pwm_set_gpio_level(PIN_MOTOR_L_PWM, (-speed_left * PWM_WRAP) / 100);
    }

    // --- MOTEUR DROIT ---
    if (speed_right >= 0) {
        gpio_put(PIN_MOTOR_R_DIR, 1); // Marche avant
        pwm_set_gpio_level(PIN_MOTOR_R_PWM, (speed_right * PWM_WRAP) / 100);
    } else {
        gpio_put(PIN_MOTOR_R_DIR, 0); // Marche arrière
        pwm_set_gpio_level(PIN_MOTOR_R_PWM, (-speed_right * PWM_WRAP) / 100);
    }
}