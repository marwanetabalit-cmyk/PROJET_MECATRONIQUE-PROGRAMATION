#pragma once
#include <Arduino.h>

struct DistanceReadings {
    float front = -1.0f;
    float left  = -1.0f;
    float right = -1.0f;
    bool obstacle = false;
};

class UltrasonicArray {
public:
    void init();
    DistanceReadings readAll();

private:
    float readOne(uint8_t trigPin, uint8_t echoPin);
};