#include "odometry.h"
#include <math.h>

static Position current_pos = {0, 0, 0};

void odometry_init() {
    current_pos.x = 0;
    current_pos.y = 0;
    current_pos.theta = 0;
}

void odometry_update(float left_cm, float right_cm) {
    // Distance parcourue
    float distance = (left_cm + right_cm) / 2.0f;
    
    // Changement d'orientation
    float delta_theta = (right_cm - left_cm) / WHEEL_DISTANCE;
    
    // Mise à jour position
    current_pos.x += distance * cos(current_pos.theta + delta_theta/2);
    current_pos.y += distance * sin(current_pos.theta + delta_theta/2);
    current_pos.theta += delta_theta;
    
    // Normalise angle entre -π et π
    while(current_pos.theta > M_PI) current_pos.theta -= 2*M_PI;
    while(current_pos.theta < -M_PI) current_pos.theta += 2*M_PI;
}

Position odometry_get_position() {
    return current_pos;
}

float distance_to_target(float target_x, float target_y) {
    float dx = target_x - current_pos.x;
    float dy = target_y - current_pos.y;
    return sqrt(dx*dx + dy*dy);
}

float angle_to_target(float target_x, float target_y) {
    float dx = target_x - current_pos.x;
    float dy = target_y - current_pos.y;
    return atan2(dy, dx) - current_pos.theta;
}