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

void IR_SENSOR_ConfigINT(void);
int IR_SENSOR_DRDY_Status(uint8_t *status);
int CheckInterruptFlags();
int IR_SENSOR_getInt();
void IR_SENSOR_clearInt();
void IR_HandleInt();

#endif
