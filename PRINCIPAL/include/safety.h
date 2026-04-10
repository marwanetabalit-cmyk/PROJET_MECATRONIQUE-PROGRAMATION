#pragma once
#include <Arduino.h>

class SafetySystem {
public:
    void init();
    bool isStartPressed() const;
    bool isEStopPressed() const;
};