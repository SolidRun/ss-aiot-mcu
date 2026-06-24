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

/* ---------------------------------------------------------------------
 * STUCK BUS RECOVERY
 * ---------------------------------------------------------------------
 * If the external CPU master dies (power-cycle or reset) in the middle
 * of an I2C2 transaction, the STM32 I2C2 peripheral (acting as slave)
 * can be left waiting for the next byte forever. Because clock
 * stretching is enabled (NoStretchMode = DISABLE), the peripheral holds
 * SCL low while it waits — and since the master is gone, that wait
 * never ends. The bus is then stuck (SCL held low) until something
 * forces the I2C2 peripheral itself to reset.
 *
 * HAL_I2C_DeInit()/Init() alone is NOT enough — it only resets the
 * driver-level struct, not the peripheral's internal state machine.
 * __HAL_RCC_I2C2_FORCE_RESET()/RELEASE_RESET() resets the peripheral at
 * the clock-domain level, which does clear the stuck state.
 *
 * This is detected by polling the HAL I2C state from the main loop: if
 * we remain in a "busy" state for longer than any real I2C2 transaction
 * should ever take, we assume the bus is stuck and force a recovery.
 * ----------------------------------------------------------------------*/

#define I2C2_STUCK_TIMEOUT_MS 10U

static uint32_t i2c2_state_entered_flag = 0;
static uint32_t i2c2_counter_tick = 0;
static HAL_I2C_StateTypeDef i2c2_last_state = HAL_I2C_STATE_RESET;

/* Call this regularly from the main while(1) loop.
 * Detects an I2C2 peripheral stuck in a busy state for too long
 * (e.g. master died mid-transaction) and forces a recovery. */
void I2C2_CheckStuckBus(void) {
    HAL_I2C_StateTypeDef current_state = HAL_I2C_GetState(&hi2c2);

    if (current_state != i2c2_last_state) {
        i2c2_last_state = current_state;
        i2c2_state_entered_flag = 1;
        i2c2_counter_tick = 0;
        return;
    }
    if(i2c2_state_entered_flag){
    	i2c2_counter_tick ++ ;
    }
    bool is_busy_state =
        (current_state == HAL_I2C_STATE_BUSY_RX) ||
        (current_state == HAL_I2C_STATE_BUSY_TX) ||
        (current_state == HAL_I2C_STATE_BUSY_RX_LISTEN) ||
        (current_state == HAL_I2C_STATE_BUSY_TX_LISTEN);

    if (is_busy_state) {
        if ( i2c2_counter_tick  > I2C2_STUCK_TIMEOUT_MS) {
        	resetI2C2();
        	//NVIC_SystemReset();
        }
    }
}


/* I2C Slave init
 */
void I2C_Slave_Init(void) {
	HAL_I2C_EnableListen_IT(&hi2c2);
	i2c2_state_entered_flag = 0;
	i2c2_counter_tick = 0;
	i2c2_last_state = HAL_I2C_GetState(&hi2c2);
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
