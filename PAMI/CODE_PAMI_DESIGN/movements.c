#include "movements.h"
#include <math.h>

#define MOTEUR_GAUCHE slice_A
#define MOTEUR_DROIT  slice_B

// ==========================================
// NIVEAU 1 : Mouvements continus (Inchangé)
// ==========================================

void pami_avancer(int vitesse) {
    motor_set_speed(MOTEUR_GAUCHE, vitesse);
    motor_set_speed(MOTEUR_DROIT, vitesse);
}

void pami_reculer(int vitesse) {
    motor_set_speed(MOTEUR_GAUCHE, -vitesse);
    motor_set_speed(MOTEUR_DROIT, -vitesse);
}

void pami_tourner_gauche(int vitesse) {
    motor_set_speed(MOTEUR_GAUCHE, -vitesse);
    motor_set_speed(MOTEUR_DROIT, vitesse);
}

void pami_tourner_droite(int vitesse) {
    motor_set_speed(MOTEUR_GAUCHE, vitesse);
    motor_set_speed(MOTEUR_DROIT, -vitesse);
}

void pami_stopper(void) {
    motor_set_speed(MOTEUR_GAUCHE, 0);
    motor_set_speed(MOTEUR_DROIT, 0);
}

// ==========================================
// OUTIL INTERNE : Boucle d'attente sur encodeurs
// ==========================================

// Cette fonction lit les encodeurs et attend qu'ils atteignent une valeur cible.
// C'est un asservissement "On/Off" très basique (pas de PID pour l'instant).
static void wait_target(int32_t target_left, int32_t target_right) {
    bool left_ok = false;
    bool right_ok = false;

    // On boucle tant que les deux moteurs n'ont pas atteint leur cible
    while (!left_ok || !right_ok) {
        
        // On récupère la valeur absolue de la progression (car on peut reculer)
        if (abs(encoder_left_pos) >= abs(target_left)) {
            motor_set_speed(MOTEUR_GAUCHE, 0); // Stoppe le moteur s'il a fini
            left_ok = true;
        }
        
        if (abs(encoder_right_pos) >= abs(target_right)) {
            motor_set_speed(MOTEUR_DROIT, 0);
            right_ok = true;
        }
        
        sleep_ms(5); // Petite pause pour ne pas saturer le CPU
    }
}

// ==========================================
// NIVEAU 2 : Mouvements précis
// ==========================================

void pami_avancer_distance(float distance_cm, int vitesse) {
    // 1. Calcul du nombre de tics nécessaires
    int32_t tics_cible = (int32_t)(distance_cm * TICS_PAR_CM);

    // 2. Remise à zéro des variables de position (IMPORTANT)
    // ATTENTION : Cela suppose que tu remets à zero tes variables globales.
    // L'idéal serait d'avoir une fonction `encoder_reset()` dans encoders.c
    encoder_left_pos = 0;
    encoder_right_pos = 0;

    // 3. Lancer les moteurs
    if (distance_cm > 0) {
        pami_avancer(vitesse);
    } else {
        pami_reculer(vitesse);
    }

    // 4. Attendre d'atteindre la cible
    wait_target(tics_cible, tics_cible);
    pami_stopper();
}

void pami_tourner_angle(float angle_degres, int vitesse) {
    // Pour tourner sur place, chaque roue parcourt un arc de cercle.
    // Le périmètre du cercle de rotation est : PI * ENTRAXE
    // La distance pour un angle donné est : (Angle / 360) * (PI * ENTRAXE)
    
    float distance_arc_cm = (angle_degres / 360.0f) * (3.14159f * ENTRAXE_CM);
    int32_t tics_cible = (int32_t)(distance_arc_cm * TICS_PAR_CM);

    encoder_left_pos = 0;
    encoder_right_pos = 0;

    if (angle_degres > 0) {
        // Tourne à droite
        pami_tourner_droite(vitesse);
    } else {
        // Tourne à gauche
        pami_tourner_gauche(vitesse);
    }

    // Un moteur avance (cible positive), l'autre recule (cible "négative" en tics, mais gérée par abs() dans wait_target)
    wait_target(tics_cible, tics_cible);
    pami_stopper();
}