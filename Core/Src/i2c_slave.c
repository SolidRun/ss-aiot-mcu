// SPDX-License-Identifier: BSD-3-Clause
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

/* The receive path arms whatever data_len asks for, so the buffer has to cover
 * the field's whole range. */
_Static_assert(I2C_CMD_MAX_PAYLOAD >= UINT8_MAX, "rxBuffer cannot hold every data_len a master can send");

static uint8_t rxBuffer[I2C_CMD_MAX_LEN];
static I2C_Command_t *rxCommand = (void *)rxBuffer;
static uint8_t txBuffer[I2C_RESP_MAX_LEN];
static I2C_Response_t *txResponse = (void *)txBuffer;

/* number of bytes received since last i2c address match */
static size_t rxcount = 0;
/* number of bytes the slave was last armed for */
static size_t expected_bytes = I2C_CMD_MIN_LEN;

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
 * resetI2C2() recovers it with HAL_I2C_DeInit() followed by
 * MX_I2C2_Init(). Both clear the PE bit in CR1, and for this I2C IP
 * clearing PE is the documented software reset: it returns the internal
 * state machine and status bits to their reset values and releases SCL
 * and SDA. __HAL_RCC_I2C2_FORCE_RESET() is not required.
 *
 * This is detected by polling the HAL I2C state from the TIM6 interrupt
 * handler, which fires at 1 Hz: if we remain in a "busy" state for more
 * ticks than any real I2C2 transaction should need, we assume the bus is
 * stuck and force a recovery. Because the tick is 1 Hz, the timeout is
 * effectively counted in seconds, not milliseconds.
 * ----------------------------------------------------------------------*/

#define I2C2_STUCK_TIMEOUT_MS 10U

static uint32_t i2c2_state_entered_flag = 0;
static uint32_t i2c2_counter_tick = 0;
static HAL_I2C_StateTypeDef i2c2_last_state = HAL_I2C_STATE_RESET;

/* Called from the TIM6 interrupt handler (1 Hz).
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
    // Process the command
    Protocol_ProcessCommand(rxCommand, txResponse);

    /*
     * ProcessCommand is responsible for initialising txResponse members
     * status, data_len, and data up to data_len bytes.
     * Where data_len < I2C_RESP_MAX_PAYLOAD remaining space may be left
     * uninitialised with stale / invalid data. The master must ensure
     * reading correct size, and discard bytes beyond data_len.
     */
}

/* HAL callback: Address match interrupt
 * Called when Master addresses this Slave.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (hi2c->Instance == hi2c2.Instance) {
        if (TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            /* Master will send data to slave, arm for size of command without payload */
            rxcount = 0;
            expected_bytes = I2C_CMD_MIN_LEN;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, expected_bytes , I2C_LAST_FRAME);
        }
        else
        {
            /*
             * Master requests data from slave.
             *
             * Arm at full buffer size allowing any size reads without
             * stalling the bus (up to sizeof tx buffer). This ensures
             * neither short nor long reads stall the bus.
             */
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_LAST_FRAME);
        }
    }
}

/* HAL callback: receive complete
 * Called when Master finishes sending data.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c2.Instance) {
        /* when we get here we have received exactly the armed number of bytes, count them */
        rxcount += expected_bytes;

        if (rxcount == I2C_CMD_MIN_LEN) {
            /* we received just received the header, no data (or stop) yet */
            if (rxCommand->data_len > 0) {
                /* prepare to receive payload data exact length */
                expected_bytes = rxCommand->data_len;

                /* arm to receive data payload */
                HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxCommand->data, expected_bytes, I2C_LAST_FRAME);
                return; // Wait for payload completion
            }
        }

        /* we have received the full command (header + payload) */
        I2C_Slave_Process();  // Process command
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
