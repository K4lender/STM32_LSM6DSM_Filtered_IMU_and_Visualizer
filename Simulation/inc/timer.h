#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// Initialize timer
void Timer_Init(void);

// Get elapsed time in milliseconds
uint32_t Timer_GetTick_ms(void);

// Get elapsed time in microseconds
uint64_t Timer_GetTick_us(void);

// Delay function (ms)
void Timer_Delay_ms(uint32_t ms);

// Elapsed time between two timestamps
uint32_t Timer_Elapsed(uint32_t start, uint32_t end);

// Check timer period (non-blocking)
int Timer_CheckPeriod(uint32_t* last_time, uint32_t period_ms);

#endif /* TIMER_H */

