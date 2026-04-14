#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

void encoders_init(void);
int32_t encoders_get_ticks_left(void);
int32_t encoders_get_ticks_right(void);
void encoders_reset(void);

#endif // ENCODERS_H