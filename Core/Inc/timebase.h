/*
 * timebase.h
 *
 * Monotonic 32-bit tick counter, with no calendar meaning.
 */

#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/*
 * LSE at 32768 Hz through the LPTIM2 prescaler at /8, as MX_LPTIM2_Init()
 * configures it. 244.140625us per tick, an exact binary fraction of a second,
 * so scaling ticks to other units needs no division.
 *
 * Two properties SysTick does not have. The LSE crystal is 20 ppm, against the
 * HSI16's 15.88-16.08 MHz at 30 degC plus a further +-1 % over temperature.
 * And LPTIM keeps counting in the low-power modes that stop the core clock.
 */
#define TIMEBASE_HZ  4096U

/* Start the counter. Call once, after MX_LPTIM2_Init(). */
void Timebase_Init(void);

/*
 * Ticks since Timebase_Init(), wrapping every 2^32 ticks - 291 hours - so an
 * unsigned difference is correct across the wrap. Callable from any context.
 */
uint32_t Timebase_Now(void);

#endif
