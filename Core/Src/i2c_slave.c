/*
 * i2c_slave.c
 *
 *  Created on: Sep 25, 2025
 *      Author: User
 */
#include "i2c_slave.h"
#include "stm32u0xx_hal.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

#define I2C_SLAVE_ADDR 0x18

static I2C_Command_t rxCommand;
static I2C_Response_t txResponse;

static uint8_t rxBuffer[sizeof(I2C_Command_t)];
static uint8_t txBuffer[sizeof(I2C_Response_t)];
static uint8_t txDataLen = 2;
static volatile bool txReady = false;
static volatile bool i2cBusy = false;

uint8_t rxcount = 0;
uint8_t expected_bytes = 3;

/* I2C Slave init
 */
void I2C_Slave_Init(void) {
	HAL_I2C_EnableListen_IT(&hi2c2);
}

/* Called when Master sends data
 * Decodes command and prepares a response.
 */
void I2C_Slave_Process(void) {
    // Copy RX buffer into command struct
    memcpy(&rxCommand, rxBuffer, sizeof(I2C_Command_t));

    // Process the command
    Protocol_ProcessCommand(&rxCommand, &txResponse);

    // Copy response struct into TX buffer
    memcpy(txBuffer, &txResponse, sizeof(I2C_Response_t));
    txDataLen = txResponse.data_len;
    txReady = true;
}

/* HAL callback: Address match interrupt
 * Called when Master addresses this Slave.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (hi2c->Instance == hi2c2.Instance) {
        if (TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            // Master will send data to slave
            rxcount = 0;
            expected_bytes = 3;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, expected_bytes , I2C_LAST_FRAME);
        }
        else
        {
            // Master requests data from slave
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, 2 + txDataLen, I2C_LAST_FRAME);
        }
    }
}

/* HAL callback: receive complete
 * Called when Master finishes sending data.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c2.Instance) {
        rxcount += expected_bytes;  // Update the number of bytes received
        // First, check if we just received the header (first 3 bytes)
        if (rxcount == 3) {
            uint8_t data_len = rxBuffer[2]; // Header's data_len
            if (data_len > 0) {
                expected_bytes = data_len;
                HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer + rxcount, expected_bytes, I2C_LAST_FRAME);
                return;  // Wait for payload completion
            }
        }

        // If we reached here, we have the full command (header + payload)
        I2C_Slave_Process();  // Process command
        i2cBusy = false;
    }
}

/* HAL callback: listen complete
 * Re-enable listening after transaction finishes.
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c2.Instance) {
    		HAL_I2C_EnableListen_IT(hi2c);
    }
}

