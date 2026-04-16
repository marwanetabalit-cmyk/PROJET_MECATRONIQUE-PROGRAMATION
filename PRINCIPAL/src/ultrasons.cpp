#include "ultrasons.h"  // Inclusion du header pour la classe UltrasonicArray
#include "config.h"    // Inclusion du header pour les configurations

// Initialise le tableau de capteurs ultrasons : configure les pins Trig et Echo
void UltrasonicArray::init() {
    // Configure les pins Trig en sortie (pour envoyer les impulsions)
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);

    // Configure les pins Echo en entrée (pour recevoir les échos)
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    // Met les pins Trig à LOW initialement
    digitalWrite(US_FRONT_TRIG, LOW);
    digitalWrite(US_LEFT_TRIG, LOW);
    digitalWrite(US_RIGHT_TRIG, LOW);
}

// Lit la distance d'un seul capteur ultrason : envoie une impulsion et mesure l'écho
float UltrasonicArray::readOne(uint8_t trigPin, uint8_t echoPin) {
    // Prépare l'impulsion : Trig à LOW pendant 2 µs
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Envoie l'impulsion : Trig à HIGH pendant 10 µs
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Mesure la durée de l'écho sur la pin Echo, avec timeout
    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASON_TIMEOUT_US);
    if (duration == 0) {
        return -1.0f;  // Retourne -1 si timeout (pas d'écho)
    }

    // Calcule la distance en cm : durée * vitesse du son / 2 (aller-retour)
    float distanceCm = (duration * SOUND_SPEED_CM_PER_US) / 2.0f;
    return distanceCm;
}

// Lit les distances de tous les capteurs et détecte les obstacles
DistanceReadings UltrasonicArray::readAll() {
    DistanceReadings d;  // Structure pour stocker les lectures

    // Lit la distance frontale
    d.front = readOne(US_FRONT_TRIG, US_FRONT_ECHO);
    delay(10);  // Petit délai pour éviter les interférences
    // Lit la distance gauche
    d.left  = readOne(US_LEFT_TRIG, US_LEFT_ECHO);
    delay(10);
    // Lit la distance droite
    d.right = readOne(US_RIGHT_TRIG, US_RIGHT_ECHO);

    // Détecte les obstacles : vrai si distance valide et inférieure au seuil
    bool frontObstacle = (d.front > 0.0f && d.front < OBSTACLE_STOP_CM);
    bool leftObstacle  = (d.left  > 0.0f && d.left  < OBSTACLE_STOP_CM);
    bool rightObstacle = (d.right > 0.0f && d.right < OBSTACLE_STOP_CM);

    // Obstacle global : vrai si au moins un capteur détecte un obstacle
    d.obstacle = frontObstacle || leftObstacle || rightObstacle;
    return d;  // Retourne la structure avec toutes les données
}