#include "drive.h"
#include "config.h"

// ============================================================================
// CONSTRUCTEUR ET INITIALISATION
// ============================================================================

/// Constructeur: sauvegarde la référence au port série et initialise l'objet Dynamixel
DriveBase::DriveBase(HardwareSerial& serialPort)
    : serial(serialPort), dxl(serialPort, DXL_DIR_PIN) {}

// ============================================================================
// UTILITAIRES DE CONFIGURATION
// ============================================================================

/// Applique l'inversion de moteur si nécessaire
/// @param rpm Vitesse demandée
/// @param inverted true si le moteur doit être inversé
/// @return Vitesse signée appropriée
float DriveBase::applyMotorSign(float rpm, bool inverted) {
    return inverted ? -rpm : rpm;
}

/// Configure un moteur Dynamixel en mode vitesse
/// @param id ID du moteur Dynamixel (1=gauche, 2=droite)
void DriveBase::configureMotor(uint8_t id) {
    dxl.torqueOff(id);                    // Désactiver le couple (pour la config)
    dxl.setOperatingMode(id, OP_VELOCITY); // Mode vitesse (pas de position cible)
    dxl.torqueOn(id);                     // Réactiver le couple
}

// ============================================================================
// INITIALISATION DU SYSTÈME DE LOCOMOTION
// ============================================================================

void DriveBase::init() {
    // Initialiser la communication série avec les moteurs
    // ESP32 Serial 2: RX=GPIO16, TX=GPIO17, à 57600 baud
    serial.begin(DXL_BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);

    // Initialiser le contrôleur Dynamixel
    dxl.begin(DXL_BAUDRATE);

    // Définir la version du protocole Dynamixel (v2.0 pour XM430/XL430)
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // Configurer chaque moteur
    configureMotor(DXL_LEFT_ID);   // Moteur gauche (ID=1)
    configureMotor(DXL_RIGHT_ID);  // Moteur droit (ID=2)

    // Arrêter les moteurs au démarrage
    stop();
}

// ============================================================================
// CONTRÔLE DE VITESSE (BAS NIVEAU)
// ============================================================================

/// Fixe la vitesse des deux moteurs indépendamment
/// @param leftRpm Vitesse moteur gauche (RPM)
/// @param rightRpm Vitesse moteur droit (RPM)
void DriveBase::setVelocity(float leftRpm, float rightRpm) {
    // Appliquer les inversions de moteur (correction des sens)
    float leftCmd  = applyMotorSign(leftRpm, LEFT_MOTOR_INVERTED);
    float rightCmd = applyMotorSign(rightRpm, RIGHT_MOTOR_INVERTED);

    // Envoyer les commandes aux moteurs (via protocole Dynamixel)
    dxl.setGoalVelocity(DXL_LEFT_ID, leftCmd, UNIT_RPM);
    dxl.setGoalVelocity(DXL_RIGHT_ID, rightCmd, UNIT_RPM);
}

// ============================================================================
// CONTRÔLE HAUT NIVEAU (MOUVEMENTS)
// ============================================================================

/// Arrête complètement le robot en mettant les vitesses à zéro
void DriveBase::stop() {
    setVelocity(0.0f, 0.0f);
}

/// Fait avancer le robot en ligne droite
/// @param rpm Vitesse d'avance (RPM)
void DriveBase::forward(float rpm) {
    setVelocity(rpm, rpm);  // Deux moteurs à la même vitesse positive
}

/// Fait reculer le robot en ligne droite
/// @param rpm Vitesse de recul (RPM)
void DriveBase::backward(float rpm) {
    setVelocity(-rpm, -rpm);  // Deux moteurs à la même vitesse négative
}

/// Fait tourner le robot vers la gauche (rotation en place)
/// @param rpm Vitesse angulaire (RPM)
void DriveBase::rotateLeft(float rpm) {
    setVelocity(-rpm, rpm);  // Moteur gauche négatif, droit positif
}

/// Fait tourner le robot vers la droite (rotation en place)
/// @param rpm Vitesse angulaire (RPM)
void DriveBase::rotateRight(float rpm) {
    setVelocity(rpm, -rpm);  // Moteur gauche positif, droit négatif
}