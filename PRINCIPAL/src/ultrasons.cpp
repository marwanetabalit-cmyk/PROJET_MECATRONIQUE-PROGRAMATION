#include "ultrasons.h"
#include "config.h"

// ============================================================================
// INITIALISATION
// ============================================================================

void UltrasonicArray::init() {
    // Configure les pins Trig en sortie (envoient les impulsions)
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);

    // Configure les pins Echo en entrée (reçoivent les échos)
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    // Initialise les pins Trig en LOW (pas d'impulsion)
    digitalWrite(US_FRONT_TRIG, LOW);
    digitalWrite(US_LEFT_TRIG, LOW);
    digitalWrite(US_RIGHT_TRIG, LOW);

    nextRead = ReadState::FRONT;
    lastReadMs = millis();
}

// ============================================================================
// LECTURE UNITAIRE NON-BLOQUANTE
// ============================================================================

float UltrasonicArray::readOne(uint8_t trigPin, uint8_t echoPin) {
    // Étape 1: Préparer l'impulsion (Trig bas pendant 2µs)
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Étape 2: Envoyer l'impulsion (Trig haut pendant 10µs)
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Étape 3: Mesurer l'écho avec timeout
    // pulseIn attend le passage du signal à HIGH, puis mesure sa durée
    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASON_TIMEOUT_US);

    // Timeout détecté (pas d'objet, trop loin, ou erreur)
    if (duration == 0) {
        return -1.0f;
    }

    // Calcul de distance: durée * vitesse du son / 2 (aller-retour)
    // Vitesse du son = 0.0343 cm/µs
    float distanceCm = (duration * SOUND_SPEED_CM_PER_US) / 2.0f;
    return distanceCm;
}

// ============================================================================
// LECTURE GLOBALE NON-BLOQUANTE
// ============================================================================

DistanceReadings UltrasonicArray::readAll() {
    unsigned long now = millis();

    // Vérifier que suffisamment de temps s'est écoulé depuis la dernière lecture
    // Cela évite les interférences entre capteurs ultrasons
    if (now - lastReadMs < MIN_READ_INTERVAL_MS) {
        // Pas prêt à lire, retourner le cache
        DistanceReadings d;
        d.front = cachedFront;
        d.left = cachedLeft;
        d.right = cachedRight;

        // Déterminer s'il y a un obstacle selon les lectures en cache
        bool frontObstacle = (d.front > 0.0f && d.front < OBSTACLE_STOP_CM);
        bool leftObstacle = (d.left > 0.0f && d.left < OBSTACLE_STOP_CM);
        bool rightObstacle = (d.right > 0.0f && d.right < OBSTACLE_STOP_CM);
        d.obstacle = frontObstacle || leftObstacle || rightObstacle;

        return d;
    }

    lastReadMs = now;

    // Machine à états: lire un capteur à la fois
    // Cela éclale les lectures pour éviter l'interférence et le blocage prolongé
    switch (nextRead) {
        case ReadState::FRONT:
            cachedFront = readOne(US_FRONT_TRIG, US_FRONT_ECHO);
            nextRead = ReadState::LEFT;
            break;

        case ReadState::LEFT:
            cachedLeft = readOne(US_LEFT_TRIG, US_LEFT_ECHO);
            nextRead = ReadState::RIGHT;
            break;

        case ReadState::RIGHT:
            cachedRight = readOne(US_RIGHT_TRIG, US_RIGHT_ECHO);
            nextRead = ReadState::FRONT;  // Recommencer le cycle
            break;
    }

    // Construire la structure de résultat avec les dernières valeurs valides
    DistanceReadings d;
    d.front = cachedFront;
    d.left = cachedLeft;
    d.right = cachedRight;

    // Détecter les obstacles:
    // Un obstacle est présent si au moins un capteur mesure une distance valide
    // ET cette distance est inférieure au seuil OBSTACLE_STOP_CM
    bool frontObstacle = (d.front > 0.0f && d.front < OBSTACLE_STOP_CM);
    bool leftObstacle = (d.left > 0.0f && d.left < OBSTACLE_STOP_CM);
    bool rightObstacle = (d.right > 0.0f && d.right < OBSTACLE_STOP_CM);

    // L'obstacle global est la combinaison logique OU des 3 capteurs
    d.obstacle = frontObstacle || leftObstacle || rightObstacle;

    return d;
}