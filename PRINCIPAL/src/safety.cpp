#include "safety.h"
#include "config.h"

void SafetySystem::init() {
    pinMode(PIN_START_SWITCH, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);
}

bool SafetySystem::isStartPressed() const {
    return digitalRead(PIN_START_SWITCH) == LOW; // actif bas
}

bool SafetySystem::isEStopPressed() const {
    return digitalRead(PIN_ESTOP) == LOW; // actif bas
}