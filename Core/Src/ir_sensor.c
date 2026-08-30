/**
 * @file    ir_sensor.c
 * @brief   IR Sensor (STHS34PF80) driver wrapper for STM32U03
 *          Includes init, WhoAmI check, continuous mode, and raw value reads.
 */

#include "ir_sensor.h"
#include "sths34pf80_reg.h"
#include "main.h" // For hi2c1, HAL_Delay
#include "stdbool.h"

extern volatile uint8_t IR_INT;
extern I2C_HandleTypeDef hi2c1;  // CubeMX I2C handle
//------------------------------------------------------------------------------
// Private variables
//------------------------------------------------------------------------------
static stmdev_ctx_t ir_sensor_ctx;
#define IR_THS_DEFAULT  1000U
uint16_t ir_ths = IR_THS_DEFAULT;

//------------------------------------------------------------------------------
// Private functions
//------------------------------------------------------------------------------
static int32_t ir_sensor_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    return (int32_t)HAL_I2C_Mem_Write(handle, STHS34PF80_I2C_ADD, reg,
                                      I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY);
}

static int32_t ir_sensor_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, STHS34PF80_I2C_ADD, reg,
                                     I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
	return (status);
}

//------------------------------------------------------------------------------
// Public functions
//------------------------------------------------------------------------------

/**
 * @brief Initialize IR Sensor context
 */
void IR_SENSOR_InitCtx()
{
    ir_sensor_ctx.write_reg = ir_sensor_write;
    ir_sensor_ctx.read_reg  = ir_sensor_read;
    ir_sensor_ctx.handle    = &hi2c1;  // I2C handle from CubeMX
    ir_sensor_ctx.mdelay    = HAL_Delay;

    sths34pf80_int_mode_t int_mode_cfg;
    int_mode_cfg.pin = STHS34PF80_PUSH_PULL;
    int_mode_cfg.polarity = STHS34PF80_ACTIVE_HIGH;
    sths34pf80_int_mode_set(&ir_sensor_ctx, int_mode_cfg);

    //sths34pf80_int_or_set(&ir_sensor_ctx, STHS34PF80_INT_MOTION);
    sths34pf80_int_or_set(&ir_sensor_ctx, STHS34PF80_INT_MOTION_PRESENCE);
    sths34pf80_presence_threshold_set(&ir_sensor_ctx, ir_ths);
    sths34pf80_motion_threshold_set(&ir_sensor_ctx, ir_ths);
    sths34pf80_route_int_set(&ir_sensor_ctx, STHS34PF80_INT_OR);
    sths34pf80_tobject_algo_compensation_set(&ir_sensor_ctx, 1);

}

/**
 * @brief Check if sensor is connected
 * @return 0 if OK, else error
 */
int IR_SENSOR_CheckConnection(void)
{
    uint8_t who_am_i;
    if (sths34pf80_device_id_get(&ir_sensor_ctx, &who_am_i) != 0)
        return -1;

    if (who_am_i != STHS34PF80_ID)
        return -2;

    return 0;
}

/**
 * @brief Configure sensor for continuous measurement
 * @param odr_hz Desired output data rate (use enum sths34pf80_odr_t)
 */
void IR_SENSOR_StartContinuous(sths34pf80_odr_t odr)
{
    sths34pf80_odr_set(&ir_sensor_ctx, odr);
    sths34pf80_block_data_update_set(&ir_sensor_ctx, PROPERTY_ENABLE);
}

/**
 * @brief Read raw object temperature
 */
int IR_SENSOR_ReadTObject(int16_t *value)
{
    return sths34pf80_tobject_raw_get(&ir_sensor_ctx, value);
}

/**
 * @brief Read raw ambient temperature
 */
int IR_SENSOR_ReadTAmbient(int16_t *value)
{
    return sths34pf80_tambient_raw_get(&ir_sensor_ctx, value);
}

/**
 * @brief Read raw presence
 */
int IR_SENSOR_ReadPresence(int16_t *value)
{
    return sths34pf80_tpresence_raw_get(&ir_sensor_ctx, value);
}

/**
 * @brief Read raw motion
 */
int IR_SENSOR_ReadMotion(int16_t *value)
{
    return sths34pf80_tmotion_raw_get(&ir_sensor_ctx, value);
}

/**
 * @brief Read raw ambient shock
 */
int IR_SENSOR_ReadTAmbShock(int16_t *value)
{
    return sths34pf80_tamb_shock_raw_get(&ir_sensor_ctx, value);
}

/**
 * @brief Configure INT pin
 */
void IR_SENSOR_ConfigINT(void)
{
    sths34pf80_int_mode_t int_cfg;
    int_cfg.pin = STHS34PF80_PUSH_PULL;
    int_cfg.polarity = STHS34PF80_ACTIVE_HIGH;
    sths34pf80_int_mode_set(&ir_sensor_ctx, int_cfg);
}

/**
 * @brief Read DRDY status
 */
int IR_SENSOR_DRDY_Status(uint8_t *status)
{
    sths34pf80_drdy_status_t drdy;
    if (sths34pf80_drdy_status_get(&ir_sensor_ctx, &drdy) != 0)
        return -1;

    *status = drdy.drdy;
    return 0;
}

/**
 * @brief Read the interrupt flags the sensor is currently asserting.
 * @retval >=0  bitmask of routed FUNC_STATUS bits (0 = nothing pending)
 * @retval  -1  the read failed - the sensor state is unknown
 *
 * Returns a mask rather than a single value: presence and motion can be set
 * together, and the SOM demultiplexes them by bit.
 */
int CheckInterruptFlags()
{
	uint8_t func_status;
    static const uint8_t mask = 0x02U | 0x04U; // motion, presence

	if (sths34pf80_read_reg(&ir_sensor_ctx, STHS34PF80_FUNC_STATUS,
	                        &func_status, 1) != 0) {
		return -1;
	}

	return (int)(func_status & mask);
}

/* get previously stored interrupt */
int IR_SENSOR_getInt()
{
	return IR_INT;
}

/* clear previously stored interrupt */
void IR_SENSOR_clearInt()
{
	IR_INT = 0;
}

/**
 * @brief Sample the sensor and notify the SOM if anything is pending.
 *
 * Called from the EXTI handler, and once directly after
 * GPIO_EnableSensorInterrupts() - the INT pin is push-pull and level-driven,
 * so a condition already asserted when the line comes up produces no rising
 * edge at all and would otherwise never be reported.
 *
 * Stores interrupt reason for later use.
 */
void IR_HandleInt()
{
	int flags = CheckInterruptFlags();

    /* abort on error */
	if (flags < 0)
		return;

    /* accumulate interrupts */
	IR_INT |= (uint8_t)flags;

    /* report any interrupts to som */
    if (IR_INT) {
	    SomEnable();
	    somSetInt();
    }
}
