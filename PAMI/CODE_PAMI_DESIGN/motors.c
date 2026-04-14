#include "motors.h"
#include "hardware/gpio.h"

// Broches GPIO pour les moteurs
#define IA1_PIN 21
#define IA2_PIN 20
#define IB1_PIN 19
#define IB2_PIN 18

uint slice_A;
uint slice_B;

// Initialisation des moteurs
void motor_init() {
    // Configuration PWM moteur A
    gpio_init(IA1_PIN);
    gpio_init(IA2_PIN);
    gpio_set_function(IA1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(IA2_PIN, GPIO_FUNC_PWM);

    pwm_config config_A = pwm_get_default_config();
    pwm_config_set_wrap(&config_A, 10000);
    slice_A = pwm_gpio_to_slice_num(IA1_PIN);
    pwm_init(slice_A, &config_A, true);

    // Configuration PWM moteur B
    gpio_init(IB1_PIN);
    gpio_init(IB2_PIN);
    gpio_set_function(IB1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(IB2_PIN, GPIO_FUNC_PWM);

    pwm_config config_B = pwm_get_default_config();
    pwm_config_set_wrap(&config_B, 10000);
    slice_B = pwm_gpio_to_slice_num(IB1_PIN);
    pwm_init(slice_B, &config_B, true);

    // Moteurs à l’arrêt au départ
    pwm_set_chan_level(slice_A, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_A, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_B, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_B, PWM_CHAN_B, 0);
}

// Définit la vitesse d’un moteur (-100 à 100)
void motor_set_speed(uint slice, int speed) {
    uint16_t level;

    // Calcul du niveau PWM en fonction de la vitesse
    if (speed > 0) level = (100 - speed) * 100;
    else if (speed < 0) level = (100 + speed) * 100;
    else level = 0;

    // Direction et vitesse du moteur
    if (speed > 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, 10000);
        pwm_set_chan_level(slice, PWM_CHAN_B, level);
    } else if (speed < 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, level);
        pwm_set_chan_level(slice, PWM_CHAN_B, 10000);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}
