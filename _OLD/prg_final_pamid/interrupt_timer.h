#ifndef INTERRUPT_TIMER_H
#define INTERRUPT_TIMER_H

#include <stdint.h>
#include <stdbool.h>

// Fréquence de l'interruption principale (Hz)
#define MAIN_INTERRUPT_FREQ 50  // 50Hz = 20ms

// Fonction callback (à définir dans main.cpp)
typedef void (*interrupt_callback_t)(void);

// Initialise le système d'interruptions
void interrupt_system_init(interrupt_callback_t callback);

// Active/désactive les interruptions globalement
void interrupts_enable(bool enable);

// Timer d'interruption simple
void interrupt_timer_start(uint32_t us_delay);

#endif