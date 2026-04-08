#include "interrupt_timer.h"
#include "hardware/timer.h"
#include "hardware/structs/timer.h"
#include "hardware/regs/timer.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

// Variable globale pour le callback
static interrupt_callback_t user_callback = NULL;

// Handler d'interruption TIMER
void __not_in_flash_func(timer_interrupt_handler)(void) {
    // Réarme le timer pour la prochaine interruption
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    timer_hw->alarm[0] = timer_hw->timerawl + (1000000 / MAIN_INTERRUPT_FREQ);
    
    // Appelle le callback utilisateur
    if(user_callback) {
        user_callback();
    }
}

// Initialise le système d'interruptions
void interrupt_system_init(interrupt_callback_t callback) {
    user_callback = callback;
    
    // Configure l'interruption timer
    hw_set_bits(&timer_hw->inte, 1u << 0);
    irq_set_exclusive_handler(TIMER0_IRQ_0, timer_interrupt_handler);
    irq_set_enabled(TIMER0_IRQ_0, true);
    
    // Premier déclenchement
    timer_hw->alarm[0] = timer_hw->timerawl + (1000000 / MAIN_INTERRUPT_FREQ);
}

// Active/désactive les interruptions
void interrupts_enable(bool enable) {
    if(enable) {
        irq_set_enabled(TIMER0_IRQ_0, true);
    } else {
        irq_set_enabled(TIMER0_IRQ_0, false);
    }
}

// Timer d'interruption simple (une fois)
void interrupt_timer_start(uint32_t us_delay) {
    timer_hw->alarm[1] = timer_hw->timerawl + us_delay;
    hw_set_bits(&timer_hw->inte, 1u << 1);
    irq_set_exclusive_handler(TIMER0_IRQ_1, timer_interrupt_handler);
    irq_set_enabled(TIMER0_IRQ_1, true);
}