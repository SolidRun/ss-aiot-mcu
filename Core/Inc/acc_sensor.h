/*
 * acc_sensor.h
 *
 *  Created on: Sep 17, 2025
 *      Author: User
 */

#ifndef ACC_SENSOR_H
#define ACC_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

/* called from main thread periodically */
void ACC_Process(void);

/* called from main thread as soon as possible */
void ACC_ProcessInt(void);

/* one sample of accelerometer motion data, packed to match wire format of i2c protocol */
typedef struct __attribute__((packed)) {
    /*
     * Timestamp is 25us per LSB on the MCU's own timebase, in RAM and on the
     * wire alike. Synced with the accelerometer's internal counter at the
     * start of each read from its FIFO.
     *
     * Absolute: it counts from MCU start and wraps every 29.8 hours. Take
     * differences modulo 2^32 and read them signed.
     */
    uint32_t timestamp; /* 25us per LSB, MCU timebase - see above */
    int16_t x; /* raw, 0.061 mg/LSB at +-2g */
    int16_t y; /* raw, 0.061 mg/LSB at +-2g */
    int16_t z; /* raw, 0.061 mg/LSB at +-2g */
} acc_motionsample_t;

/* Current value of the counter the timestamps above are taken from, so a
 * reader can turn one into an age. Same 25us units and same wrap. */
uint32_t ACC_TimestampNow(void);

/* Take as many samples from buffer as are available and fit destination */
size_t ACC_TakeMotionSamples(acc_motionsample_t *dst, size_t max_count);

/* one sample of die temperature data, packed to match wire format of i2c protocol */
typedef struct __attribute__((packed)) {
    uint32_t timestamp; /* 25us per LSB, MCU timebase - see above */
    int16_t temp; /* raw, 256 LSB/degC, 0 LSB at 25 degC */
} acc_tempsample_t;

/* As ACC_TakeMotionSamples, for the single cached die temperature. */
size_t ACC_TakeTempSamples(acc_tempsample_t *dst, size_t max_count);

#endif

