/*
 * acc_sensor.h
 *
 *  Created on: Sep 17, 2025
 *      Author: User
 */

#ifndef ACC_SENSOR_H
#define ACC_SENSOR_H

#include "ism330dhcx.h"

// Wake-Up Threshold defines the acceleration level that triggers a wake-up event.
// Possible threshold values: 0 – 63
// Each LSB corresponds to a fraction of the selected full-scale range:
//
//   Threshold (mg) = FS(g) × (threshold / 64) × 1000
//
// Example mapping by full scale:
//
//   | Threshold | ±2g (mg) | ±4g (mg) | ±8g (mg) | ±16g (mg) |
//   |-----------:|:----------|:----------|:----------|:-----------|
//   | 1          | 31 mg     | 62 mg     | 125 mg    | 250 mg     |
//   | 2          | 62 mg     | 125 mg    | 250 mg    | 500 mg     |
//   | 4          | 125 mg    | 250 mg    | 500 mg    | 1 g        |
//   | 8          | 250 mg    | 500 mg    | 1 g       | 2 g        |
//   | 16         | 500 mg    | 1 g       | 2 g       | 4 g        |
//   | 32         | 1 g       | 2 g       | 4 g       | 8 g        |
//   | 63         | 1.97 g    | 3.94 g    | 7.88 g    | 15.75 g    |
//
// Recommended values:
//   - 1–3  → Very sensitive (tiny motion)
//   - 4–8  → Medium motion (walking, light shake)
//   - 10–20 → Strong motion (hit, fall)



// Initialize the accelerometer
int ACC_Init(void);

// Read accelerometer axes
int ACC_ReadAxes(ISM330DHCX_Axes_t *axes);

// Enable DRDY (Data Ready) interrupt on INT1
int ACC_EnableDRDY(void);
int ACC_getInt();
void ACC_clearInt();
void ACC_HandleInt();

/* Take a fresh sample into the axis cache. This is a bus operation - only
 * ever called from the _6AX_INT EXTI handler. */
int ACC_RefreshAxes(void);

/* Serve the cached sample - xyz in mg, temp_c100 in hundredths of a degree
 * Celsius. Touches no bus. The return value is the protocol status byte:
 *   0 = the sample is no older than a second
 *   1 = nothing has ever been sampled, both outputs untouched
 *   2 = a real sample, but an older one */
int ACC_GetCachedAxes(int16_t *xyz, int16_t *temp_c100);
#endif

