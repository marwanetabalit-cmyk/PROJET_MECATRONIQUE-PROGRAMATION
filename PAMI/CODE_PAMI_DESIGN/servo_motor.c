#include "servo_motor.h"

// Valeurs minimales et maximales du signal PWM pour le servo (en microsecondes)
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

// Fonction externe qui envoie le signal PWM au servo
extern void PWM_SetPulse_us(unsigned long pulse_us);

// Initialisation du servo : le place à l’angle central (90°)
void Servo_Init(void)
{
    Servo_SetAngle(90);
}

// Définit l’angle du servo (0 à 180°)
void Servo_SetAngle(unsigned long angle)
{
    if (angle > 180) angle = 180; // Limite l’angle maximum

    // Calcul du signal PWM correspondant à l’angle
    unsigned long pulse_us =
        SERVO_MIN_PULSE_US +
        ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180;

    // Envoi du signal PWM au servo
    PWM_SetPulse_us(pulse_us);
}
