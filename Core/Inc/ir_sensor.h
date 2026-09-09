#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>

#include "sths34pf80_reg.h"

typedef struct {
    uint8_t drdy     : 1;  // Data ready
    uint8_t presence : 1;  // Presence detection
    uint8_t motion   : 1;  // Motion detection
    uint8_t tshock   : 1;  // Thermal shock detection
    uint8_t not_used : 4;
} sths34pf80_interrupt_flags_t;

void IR_SENSOR_InitCtx();
int IR_SENSOR_CheckConnection(void);
void IR_SENSOR_StartContinuous(sths34pf80_odr_t odr);

int IR_SENSOR_ReadTObject(int16_t *value);
int IR_SENSOR_ReadTAmbient(int16_t *value);
int IR_SENSOR_ReadPresence(int16_t *value);
int IR_SENSOR_ReadMotion(int16_t *value);
int IR_SENSOR_ReadTAmbShock(int16_t *value);

/* one sample of IR data, packed to match wire format of i2c protocol */
typedef struct __attribute__((packed)) {
    /*
     * Timestamp is 25us per LSB on the MCU's own timebase, taken when the
     * sample was read out of the sensor.
     *
     * Absolute: it counts from MCU start and wraps every 29.8 hours. Take
     * differences modulo 2^32 and read them signed.
     */
    uint32_t timestamp; /* 25us per LSB, MCU timebase - see above */
    int16_t presence; /* raw algorithm output */
    int16_t motion; /* raw algorithm output */
    int16_t tambient; /* raw, 100 LSB/degC, so hundredths of a degree */
} ir_sample_t;

/* Current value of the counter the timestamps above are taken from, so a
 * reader can turn one into an age. Same 25us units and same wrap. */
uint32_t IR_TimestampNow(void);

/* Take as many samples from buffer as are available and fit destination */
size_t IR_TakeSamples(ir_sample_t *dst, size_t max_count);

void IR_SENSOR_ConfigINT(void);
int IR_SENSOR_DRDY_Status(uint8_t *status);
int IR_SENSOR_getInt();
void IR_SENSOR_clearInt();
void IR_HandleInt();

#endif
