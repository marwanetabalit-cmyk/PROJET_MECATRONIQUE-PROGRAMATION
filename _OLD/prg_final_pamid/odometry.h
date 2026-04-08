#ifndef ODOMETRY_H
#define ODOMETRY_H

// Structure position
typedef struct {
    float x;      // Position X (cm)
    float y;      // Position Y (cm)
    float theta;  // Orientation (radians)
} Position;

// Constantes robot
#define WHEEL_DIAMETER 5f   // Diamètre roue (cm)
#define WHEEL_DISTANCE 14.4f  // Distance entre roues (cm)

// Initialise à la position (0,0)
void odometry_init();

// Met à jour avec déplacements (cm)
void odometry_update(float left_cm, float right_cm);

// Donne position actuelle
Position odometry_get_position();

// Calcule distance au point cible
float distance_to_target(float target_x, float target_y);

// Calcule angle vers cible
float angle_to_target(float target_x, float target_y);

#endif