/*
 * i2c_slave.h
 *
 *  Created on: Sep 25, 2025
 *      Author: User
 */

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "protocol.h"

/* Initialize I2C Slave interface */
void I2C_Slave_Init(void);

/* Call inside main loop to handle commands */
void I2C_Slave_Process(void);

void I2C2_CheckStuckBus(void);
#endif // I2C_SLAVE_H

