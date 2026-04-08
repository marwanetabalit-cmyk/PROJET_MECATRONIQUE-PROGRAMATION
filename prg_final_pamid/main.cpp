#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hal_motors.h"
#include "hal_servo.h"          // AJOUTÉ: servomoteur pour animation
#include "hal_ultrasound.h"    // Conservé mais commenté
#include "odometry.h"
#include "interrupt_timer.h"   // AJOUTÉ: interruptions hardware

// ========== CONFIGURATION ==========
#define SPEED_NORMAL 60        // Vitesse normale (0-100)
#define SPEED_ROTATE 50        // Vitesse rotation
#define DIST_OBSTACLE 5.0f     // Distance d'évitement (cm) - commenté pour l'instant
#define DIST_ARRIVEE 15.0f     // Distance d'arrivée (cm)

// Fréquence d'interruption (remplace le sleep_ms(50))
#define INTERRUPT_FREQ 20      // 20Hz = 50ms

// Point cible (à modifier selon besoin)
#define TARGET_X 70.0f         // Coordonnée X finale (cm)
#define TARGET_Y 40.0f         // Coordonnée Y finale (cm)

// ========== ÉTATS DU ROBOT ==========
typedef enum {
    STATE_INIT,           // Initialisation
    STATE_MOVE,           // Déplacement vers cible
    STATE_AVOID_LEFT,     // Évitement gauche (commenté pour l'instant)
    STATE_ARRIVED         // Arrivée à destination
} RobotState;

// Variables globales VOLATILES (partagées avec interruption)
static volatile RobotState state = STATE_INIT;
static volatile uint32_t interrupt_counter = 0;
static volatile bool state_machine_ready = false;
static volatile bool display_ready = false;

// Variables pour état ARRIVED
static volatile bool animation_started = false;
static volatile bool animation_completed = false;

// ========== FONCTION D'INTERRUPTION ==========

// Fonction appelée par l'interruption hardware (20Hz)
void interrupt_callback(void) {
    // Incrémente le compteur d'interruptions
    interrupt_counter++;
    
    // Signal que la machine à états doit s'exécuter
    state_machine_ready = true;
    
    // Signal l'affichage toutes les secondes (20 interruptions)
    if (interrupt_counter % 20 == 0) {
        display_ready = true;
    }
}

// ========== FONCTIONS D'ÉTAT ==========

// État INIT: initialise tout
void state_init() {
    printf("[INIT] Démarrage robot...\n");
    motors_init();
    servo_init();               // AJOUTÉ: initialisation servomoteur
    // ultrasound_init();       // COMMENTÉ: pas d'ultrason pour l'instant
    odometry_init();
    
    printf("[INIT] Cible: (%.1f, %.1f) cm\n", TARGET_X, TARGET_Y);
    state = STATE_MOVE;
}

// État MOVE: avance vers la cible
void state_move() {
    /*
    // COMMENTÉ: Détection d'obstacle désactivée pour l'instant
    if(obstacle_detected(DIST_OBSTACLE)) {
        motors_stop();
        printf("[AVOID] Obstacle détecté!\n");
        state = STATE_AVOID_LEFT;
        state_timer = 0;
        return;
    }
    */
    
    // Vérifie arrivée
    if(distance_to_target(TARGET_X, TARGET_Y) < DIST_ARRIVEE) {
        motors_stop();
        printf("[ARRIVED] Destination atteinte!\n");
        state = STATE_ARRIVED;
        return;
    }
    
    // Avance normalement
    motors_forward(SPEED_NORMAL);
    
    // Simule odométrie (à remplacer par vrais encodeurs)
    odometry_update(0.5f, 0.5f);  // 0.5cm par cycle
}

/*
// COMMENTÉ: État d'évitement désactivé pour l'instant
void state_avoid_left() {
    switch(state_timer) {
        case 0:  // 1. Tourne à 90° gauche
            motors_rotate_left(SPEED_ROTATE);
            break;
        case 10: // Après 500ms, avance latéralement
            motors_forward(SPEED_NORMAL);
            break;
        case 18: // Après 400ms, tourne à 90° droite
            motors_rotate_right(SPEED_ROTATE);
            break;
        case 28: // Retour à l'alignement
            motors_stop();
            state = STATE_MOVE;
            printf("[MOVE] Esquive terminée\n");
            break;
    }
    state_timer++;
}
*/

// État ARRIVED: animation finale
void state_arrived() {
    static uint8_t count = 0;
    
    if(count == 0) {
        motors_stop();
        printf("[ANIMATION] Début de l'animation servomoteur\n");
        servo_celebrate();  // AJOUTÉ: animation servomoteur
        count++;
        animation_completed = true;
    }
    
    // Reste arrêté après l'animation
}

// ========== BOUCLE PRINCIPALE ==========
int main() {
    // CORRECTION: Initialisation de la communication série USB
    stdio_init_all();  // Initialise USB et UART
    
    // Attente plus longue pour que l'USB soit prêt
    printf("\nAttente de la connexion USB...\n");
    sleep_ms(2500);  // 2.5 secondes pour être sûr
    
    printf("\n=== Robot 2 Roues - avec Interruptions ===\n");
    printf("Mode: Avance tout droit vers la cible\n");
    printf("Interruptions: ACTIVÉES à %dHz\n", INTERRUPT_FREQ);
    printf("Ultrason: DÉSACTIVÉ pour l'instant\n");
    printf("Servomoteur: ACTIVÉ pour animation d'arrivée\n");
    printf("Vitesse de communication: 115200 bauds\n");
    printf("Cible: (%.1f, %.1f) cm\n\n", TARGET_X, TARGET_Y);
    
    // Initialisation de la machine à états
    state_init();
    
    // Initialisation du système d'interruptions
    interrupt_system_init(interrupt_callback);
    
    // Boucle principale - remplace le sleep_ms(50) par attente d'interruption
    while(1) {
        // Attente active mais légère (consomme un peu de CPU)
        while(!state_machine_ready) {
            tight_loop_contents();  // Boucle optimisée
        }
        
        // Réinitialise le flag
        state_machine_ready = false;
        
        // Exécute l'état courant
        switch(state) {
            case STATE_INIT:        
                // Normalement déjà fait
                break;
            case STATE_MOVE:        
                state_move(); 
                break;
            // case STATE_AVOID_LEFT:  
            //     state_avoid_left(); 
            //     break;  // COMMENTÉ
            case STATE_ARRIVED:     
                if (!animation_completed) {
                    state_arrived(); 
                }
                break;
            default:
                printf("[ERREUR] État inconnu: %d\n", state);
                motors_stop();
                break;
        }
        
        // Affichage position (quand signalé par interruption)
        if(display_ready) {
            display_ready = false;
            
            Position pos = odometry_get_position();
            float distance = distance_to_target(TARGET_X, TARGET_Y);
            printf("Position: (%.1f, %.1f) th=%.2f - Distance: %.1f cm - Int: %lu\n", 
                   pos.x, pos.y, pos.theta, distance, interrupt_counter);
        }
        
        // REMPLACEMENT du sleep_ms(50) par l'interruption timer
        // Plus besoin de sleep! Les interruptions gèrent le timing
    }
    
    return 0;
}