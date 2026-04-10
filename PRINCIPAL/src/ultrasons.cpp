#include "ultrasons.h"
#include "config.h"

void UltrasonicArray::init() {
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);

    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    digitalWrite(US_FRONT_TRIG, LOW);
    digitalWrite(US_LEFT_TRIG, LOW);
    digitalWrite(US_RIGHT_TRIG, LOW);
}

float UltrasonicArray::readOne(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASON_TIMEOUT_US);
    if (duration == 0) {
        return -1.0f;
    }

    float distanceCm = (duration * SOUND_SPEED_CM_PER_US) / 2.0f;
    return distanceCm;
}

DistanceReadings UltrasonicArray::readAll() {
    DistanceReadings d;

    d.front = readOne(US_FRONT_TRIG, US_FRONT_ECHO);
    delay(10);
    d.left  = readOne(US_LEFT_TRIG, US_LEFT_ECHO);
    delay(10);
    d.right = readOne(US_RIGHT_TRIG, US_RIGHT_ECHO);

    bool frontObstacle = (d.front > 0.0f && d.front < OBSTACLE_STOP_CM);
    bool leftObstacle  = (d.left  > 0.0f && d.left  < OBSTACLE_STOP_CM);
    bool rightObstacle = (d.right > 0.0f && d.right < OBSTACLE_STOP_CM);

    d.obstacle = frontObstacle || leftObstacle || rightObstacle;
    return d;
}