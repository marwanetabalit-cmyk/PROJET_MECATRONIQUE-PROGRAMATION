#ifndef SERVO_H
#define SERVO_H

// Initialise le servo (place à l’angle central)
void Servo_Init(void);

// Définit l’angle du servo (0 à 180°)
void Servo_SetAngle(unsigned long angle);

#endif
