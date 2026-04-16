#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include "pico/stdlib.h"
#include "motors.h"
#include "encoders.h" // Nécessaire pour accéder aux positions

// --- Constantes physiques du robot ---
#define TICS_PAR_TOUR       700.0f  //
#define DIAMETRE_ROUE_CM    6.5f     // Diamètre de tes roues
#define ENTRAXE_CM          8.83f    // Distance entre le centre des deux roues

// Constantes calculées utiles
#define PERIMETRE_ROUE      (DIAMETRE_ROUE_CM * 3.14159f)
#define TICS_PAR_CM         (TICS_PAR_TOUR / PERIMETRE_ROUE)

// --- Mouvements basiques continus ---
void pami_avancer(int vitesse);
void pami_reculer(int vitesse);
void pami_tourner_gauche(int vitesse);
void pami_tourner_droite(int vitesse);
void pami_stopper(void);

// --- Mouvements précis (Utilisent les encodeurs) ---
void pami_avancer_distance(float distance_cm, int vitesse);
void pami_tourner_angle(float angle_degres, int vitesse);

// --- Déplacement Absolu (Odométrie simple) ---
void pami_aller_a(float x, float y, int vitesse);

#endif // MOVEMENTS_H