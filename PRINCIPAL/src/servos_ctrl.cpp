#include "servos_ctrl.h"  // Inclusion du header pour la classe ServoController
#include "config.h"      // Inclusion du header pour les configurations

// Initialise le contrôleur des servos : configure la fréquence et attache les servos aux pins
void ServoController::init() {
    // Définit la fréquence PWM à 50 Hz pour chaque servo (standard pour les servos)
    servoLift.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);
    servoSplit.setPeriodHertz(50);
    servoCursor.setPeriodHertz(50);

    // Attache chaque servo à sa pin avec les limites de pulsation (min et max en microsecondes)
    servoLift.attach(SERVO_LIFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoGrip.attach(SERVO_GRIP_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoSplit.attach(SERVO_SPLIT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoCursor.attach(SERVO_CURSOR_PIN, SERVO_MIN_US, SERVO_MAX_US);

    // Positionne les servos dans leur position initiale (descendue, ouverte, etc.)
    liftDown();
    gripOpen();
    splitOpen();
    cursorHome();
}

// Abaisse le servo de levage
void ServoController::liftDown() {
    servoLift.write(LIFT_DOWN_ANGLE);  // Écrit l'angle de descente
}

// Lève le servo de levage
void ServoController::liftUp() {
    servoLift.write(LIFT_UP_ANGLE);  // Écrit l'angle de montée
}

// Ouvre le servo de préhension
void ServoController::gripOpen() {
    servoGrip.write(GRIP_OPEN_ANGLE);  // Écrit l'angle d'ouverture
}

// Ferme le servo de préhension
void ServoController::gripClose() {
    servoGrip.write(GRIP_CLOSE_ANGLE);  // Écrit l'angle de fermeture
}

// Ouvre le servo de séparation
void ServoController::splitOpen() {
    servoSplit.write(SPLIT_OPEN_ANGLE);  // Écrit l'angle d'ouverture
}

// Ferme le servo de séparation
void ServoController::splitClose() {
    servoSplit.write(SPLIT_CLOSE_ANGLE);  // Écrit l'angle de fermeture
}

// Ramène le servo curseur à sa position d'origine
void ServoController::cursorHome() {
    servoCursor.write(CURSOR_HOME_ANGLE);  // Écrit l'angle de repos
}

// Pousse avec le servo curseur
void ServoController::cursorPush() {
    servoCursor.write(CURSOR_PUSH_ANGLE);  // Écrit l'angle de poussée
}

// Séquence de démonstration : exécute une série d'actions avec des délais
void ServoController::demoSequence() {
    splitOpen();   // Ouvre la séparation
    delay(500);    // Attend 500 ms

    gripOpen();    // Ouvre la préhension
    delay(500);    // Attend 500 ms

    liftDown();    // Abaisse le levage
    delay(500);    // Attend 500 ms

    gripClose();   // Ferme la préhension
    delay(700);    // Attend 700 ms

    liftUp();      // Lève le levage
    delay(700);    // Attend 700 ms

    splitClose();  // Ferme la séparation
    delay(500);    // Attend 500 ms

    cursorPush();  // Pousse avec le curseur
    delay(600);    // Attend 600 ms

    cursorHome();  // Ramène le curseur à l'origine
    delay(600);    // Attend 600 ms
}