#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "motors.h"
#include "encoders.h"
#include "ultrasound.h"

// --- CONFIGURATION PAMI ---
#define PIN_TIRETTE 15
#define PIN_SERVO 16
#define DISTANCE_CIBLE_TICKS 5000 // À convertir expérimentalement : 1 tick = X mm

// États de la stratégie
typedef enum {
    STATE_INIT,
    STATE_WAITING_85S,
    STATE_MOVING,
    STATE_DEPLOYING,
    STATE_END
} RobotState;

volatile RobotState current_state = STATE_INIT;
volatile uint32_t match_start_time = 0;
volatile bool objectif_atteint = false;

// --- PARAMÈTRES ASSERVISSEMENT (PID Simplifié) ---
// Ces valeurs sont à régler (tuner) sur la table avec ton vrai robot !
const float KP_DIST = 0.05; // Force d'accélération/freinage
const float KP_CAP  = 0.1;  // Force de la correction de la ligne droite
const int PWM_MAX = 70;     // Vitesse max (sur 100)
const int PWM_MIN = 25;     // Vitesse min pour ne pas bloquer

// Fonction simplifiée pour le servomoteur (Période 20ms = 50Hz)
void servo_set_angle(int pin, float angle) {
    // Pour un servo standard : 0° = 1ms, 90° = 1.5ms, 180° = 2ms de temps à l'état haut
    float duty_ms = 1.0 + (angle / 180.0);
    int level = (duty_ms / 20.0) * PWM_WRAP; // Nécessite que le PWM_WRAP du servo soit configuré à 50Hz
    pwm_set_gpio_level(pin, level);
}

// --- BOUCLE DE CONTRÔLE TEMPS RÉEL (100 Hz / toutes les 10 ms) ---
bool control_loop(struct repeating_timer *t) {
    // 1. Déclenchement de l'ultrason pour la prochaine lecture
    ultrasound_trigger();

    if (current_state == STATE_MOVING) {
        int32_t ticks_L = encoders_get_ticks_left();
        int32_t ticks_R = encoders_get_ticks_right();

        // Calcul des erreurs
        int32_t distance_actuelle = (ticks_L + ticks_R) / 2;
        int32_t erreur_distance = DISTANCE_CIBLE_TICKS - distance_actuelle;
        int32_t erreur_cap = ticks_L - ticks_R; // Si L va trop vite, erreur_cap > 0

        // Condition d'arrêt (Tolérance de 10 ticks)
        if (erreur_distance <= 10) {
            motors_set_speed(0, 0);
            objectif_atteint = true;
            return true;
        }

        // Calcul P (Proportionnel)
        float pwm_base = KP_DIST * (float)erreur_distance;
        float pwm_corr = KP_CAP * (float)erreur_cap;

        // Saturations de la base
        if (pwm_base > PWM_MAX) pwm_base = PWM_MAX;
        if (pwm_base < PWM_MIN && erreur_distance > 50) pwm_base = PWM_MIN; 

        // Mixage (Le robot dérive à droite ? L ralenti, R accélère)
        float cmd_L = pwm_base - pwm_corr;
        float cmd_R = pwm_base + pwm_corr;

        // Sécurité Ultrason (Arrêt si obstacle entre 1cm et 15cm)
        float dist_obs = ultrasound_get_distance_cm();
        if (dist_obs > 1.0 && dist_obs < 15.0) {
            motors_set_speed(0, 0);
        } else {
            motors_set_speed((int)cmd_L, (int)cmd_R);
        }
    }
    return true; 
}

// --- FONCTION PRINCIPALE ---
int main() {
    stdio_init_all();

    // Initialisations matérielles
    motors_init();
    encoders_init();
    ultrasound_init();

    // Init Tirette
    gpio_init(PIN_TIRETTE);
    gpio_set_dir(PIN_TIRETTE, GPIO_IN);
    gpio_pull_up(PIN_TIRETTE); // Active la résistance interne

    // Init Servo (Configuration PWM à 50Hz)
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    uint slice_servo = pwm_gpio_to_slice_num(PIN_SERVO);
    pwm_set_clkdiv(slice_servo, 64.0); // Diviseur pour atteindre 50Hz avec l'horloge système
    pwm_set_wrap(slice_servo, 39062);  // Wrap calculé pour 50Hz (20ms)
    pwm_set_enabled(slice_servo, true);
    servo_set_angle(PIN_SERVO, 0); // Position repos

    // Lancement du timer matériel (100 Hz = 10ms)
    struct repeating_timer timer;
    add_repeating_timer_ms(-10, control_loop, NULL, &timer);

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // COUPE-CIRCUIT DE SÉCURITÉ (Arrêt à 98 secondes)
        if (current_state > STATE_INIT && current_time - match_start_time > 98000) {
            current_state = STATE_END;
        }

        switch (current_state) {
            case STATE_INIT:
                // Attente du retrait de la tirette (Broche reliée au GND, passe à 3.3V via Pull-Up quand retirée)
                if (gpio_get(PIN_TIRETTE) == 1) { 
                    sleep_ms(50); // Anti-rebond (debounce)
                    if (gpio_get(PIN_TIRETTE) == 1) {
                        match_start_time = current_time;
                        current_state = STATE_WAITING_85S;
                    }
                }
                break;

            case STATE_WAITING_85S:
                if (current_time - match_start_time >= 85000) {
                    encoders_reset();
                    objectif_atteint = false;
                    current_state = STATE_MOVING;
                }
                break;

            case STATE_MOVING:
                // L'asservissement gère les moteurs en arrière-plan
                if (objectif_atteint) {
                    current_state = STATE_DEPLOYING;
                }
                break;

            case STATE_DEPLOYING:
                servo_set_angle(PIN_SERVO, 90); // Actionnement !
                sleep_ms(1500); // Laisse le temps au servo
                current_state = STATE_END;
                break;

            case STATE_END:
                motors_set_speed(0, 0);
                // Le robot est inactif jusqu'au reboot
                break;
        }
        
        // Boucle principale très légère
        sleep_ms(10);
    }
}