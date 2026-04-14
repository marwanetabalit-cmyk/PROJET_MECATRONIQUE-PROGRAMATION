#ifndef ULTRASOUND_H
#define ULTRASOUND_H

void ultrasound_init(void);

// Lance une impulsion de mesure (à appeler depuis le main ou un timer)
void ultrasound_trigger(void);

// Renvoie la dernière distance calculée en cm
float ultrasound_get_distance_cm(void);

#endif // ULTRASOUND_H