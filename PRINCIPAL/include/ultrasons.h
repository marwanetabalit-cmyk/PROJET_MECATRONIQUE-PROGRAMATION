#pragma once
#include <Arduino.h>

/// Structure contenant les lectures des 3 capteurs ultrasons
struct DistanceReadings {
    float front = -1.0f;   ///< Distance avant (cm), -1 si invalide
    float left  = -1.0f;   ///< Distance gauche (cm), -1 si invalide
    float right = -1.0f;   ///< Distance droite (cm), -1 si invalide
    bool obstacle = false; ///< Vrai si obstacle détecté
};

/// Classe de gestion du tableau de capteurs ultrasons
/// Utilise une machine à états interne pour éviter les delays bloquants
class UltrasonicArray {
public:
    /// Initialise les pins des capteurs ultrasons
    void init();

    /// Lit les distances de manière non-bloquante
    /// Chaque appel lit un capteur différent pour maximiser la fréquence
    /// @return Structure avec les distances valides et le statut d'obstacle
    DistanceReadings readAll();

private:
    // Machine à états pour alterner les lectures entre les 3 capteurs
    enum class ReadState {
        FRONT,  ///< Lire le capteur avant
        LEFT,   ///< Lire le capteur gauche
        RIGHT   ///< Lire le capteur droit
    };

    ReadState nextRead = ReadState::FRONT;          ///< Prochain capteur à lire
    unsigned long lastReadMs = 0;                   ///< Timestamp de la dernière lecture
    static constexpr unsigned long MIN_READ_INTERVAL_MS = 50; ///< Délai min entre lectures (évite interférences)

    // Cache des dernières lectures valides
    float cachedFront = -1.0f;
    float cachedLeft = -1.0f;
    float cachedRight = -1.0f;

    /// Lit la distance d'un capteur
    /// Mesure l'écho, gère le timeout
    /// @param trigPin Pin de déclenchement
    /// @param echoPin Pin de réception
    /// @return Distance en cm, ou -1.0f si timeout/erreur
    float readOne(uint8_t trigPin, uint8_t echoPin);
};