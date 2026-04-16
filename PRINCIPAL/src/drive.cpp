#include "drive.h"   // Inclusion du header pour la classe DriveBase
#include "config.h"   // Inclusion du header pour les configurations

// Constructeur de DriveBase : initialise la communication série et le contrôleur Dynamixel
DriveBase::DriveBase(HardwareSerial& serialPort)
    : serial(serialPort), dxl(serialPort, DXL_DIR_PIN) {}

// Applique le signe correct à la vitesse du moteur selon s'il est inversé
float DriveBase::applyMotorSign(float rpm, bool inverted) {
    return inverted ? -rpm : rpm;  // Inverse la vitesse si nécessaire
}

// Configure un moteur Dynamixel : désactive le couple, définit le mode vitesse, active le couple
void DriveBase::configureMotor(uint8_t id) {
    dxl.torqueOff(id);                    // Désactive le couple pour la configuration
    dxl.setOperatingMode(id, OP_VELOCITY); // Définit le mode de fonctionnement en vitesse
    dxl.torqueOn(id);                     // Active le couple
}

// Initialise le système de conduite : configure la communication série et les moteurs
void DriveBase::init() {
    serial.begin(DXL_BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);  // Initialise la communication série
    dxl.begin(DXL_BAUDRATE);                                         // Démarre le contrôleur Dynamixel
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);                // Définit la version du protocole

    configureMotor(DXL_LEFT_ID);   // Configure le moteur gauche
    configureMotor(DXL_RIGHT_ID);  // Configure le moteur droit

    stop();  // Arrête les moteurs après l'initialisation
}

// Définit la vitesse des moteurs gauche et droit, en appliquant les inversions si nécessaire
void DriveBase::setVelocity(float leftRpm, float rightRpm) {
    float leftCmd  = applyMotorSign(leftRpm, LEFT_MOTOR_INVERTED);   // Applique l'inversion pour le moteur gauche
    float rightCmd = applyMotorSign(rightRpm, RIGHT_MOTOR_INVERTED); // Applique l'inversion pour le moteur droit

    dxl.setGoalVelocity(DXL_LEFT_ID, leftCmd, UNIT_RPM);   // Définit la vitesse du moteur gauche
    dxl.setGoalVelocity(DXL_RIGHT_ID, rightCmd, UNIT_RPM); // Définit la vitesse du moteur droit
}

// Arrête le robot en mettant les vitesses à zéro
void DriveBase::stop() {
    setVelocity(0.0f, 0.0f);  // Vitesses nulles pour les deux moteurs
}

// Fait avancer le robot à la vitesse donnée
void DriveBase::forward(float rpm) {
    setVelocity(rpm, rpm);  // Vitesses égales et positives pour avancer
}

// Fait reculer le robot à la vitesse donnée
void DriveBase::backward(float rpm) {
    setVelocity(-rpm, -rpm);  // Vitesses égales et négatives pour reculer
}

// Fait tourner le robot vers la gauche à la vitesse donnée
void DriveBase::rotateLeft(float rpm) {
    setVelocity(-rpm, rpm);  // Moteur gauche négatif, moteur droit positif pour tourner à gauche
}

// Fait tourner le robot vers la droite à la vitesse donnée
void DriveBase::rotateRight(float rpm) {
    setVelocity(rpm, -rpm);  // Moteur gauche positif, moteur droit négatif pour tourner à droite
}