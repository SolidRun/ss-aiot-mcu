/**
 * @file    ir_sensor.c
 * @brief   IR Sensor (STHS34PF80) driver wrapper for STM32U03
 *          Includes init, WhoAmI check, continuous mode, and raw value reads.
 */

#include "ir_sensor.h"
#include "sths34pf80_reg.h"
#include "main.h" // For hi2c1, HAL_Delay
#include "stdbool.h"
#include "circular_buffer.h"
#include "timebase.h"

extern volatile uint8_t IR_INT;
extern I2C_HandleTypeDef hi2c1;  // CubeMX I2C handle
//------------------------------------------------------------------------------
// Private variables
//------------------------------------------------------------------------------
static stmdev_ctx_t ir_sensor_ctx;
#define IR_THS_DEFAULT  1000U
#define IR_I2C_TIMEOUT_MS   100U
uint16_t ir_ths = IR_THS_DEFAULT;

/* Samples are stamped 25us per LSB, so 40000 of them to the second. */
#define IR_TS_LSB_PER_SEC   40000U

/* IR_TicksTo25us() below hardcodes the ratio of these two constants as
 * 625/64 instead of referencing them, so pin the contract. */
_Static_assert(IR_TS_LSB_PER_SEC * 64U == TIMEBASE_HZ * 625U, "625/64 is no longer IR_TS_LSB_PER_SEC / TIMEBASE_HZ");

/* Restate an MCU tick count in 25us units, scaling by 625/64 modulo 2^32.
 * Truncates by at most 1 LSB. */
static uint32_t IR_TicksTo25us(uint32_t ticks)
{
    return ((ticks >> 6) * 625U) + (((ticks & 0x3FU) * 625U) >> 6);
}

/* Allocate circular buffer for samples. Enough for 1s at ODR 30Hz or 30s at ODR 1Hz. */
#define IR_SAMPLE_BUF_SIZE 30U
static cbuf_handle_t ir_sample_cbuf;
static struct circular_buf_t ir_sample_cbuf_priv;
static uint8_t ir_sample_cbuf_stor[IR_SAMPLE_BUF_SIZE * sizeof(ir_sample_t)];

//------------------------------------------------------------------------------
// Private functions
//------------------------------------------------------------------------------
static int32_t ir_sensor_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    return (int32_t)HAL_I2C_Mem_Write(handle, STHS34PF80_I2C_ADD, reg,
                                      I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, IR_I2C_TIMEOUT_MS);
}

static int32_t ir_sensor_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, STHS34PF80_I2C_ADD, reg,
                                     I2C_MEMADD_SIZE_8BIT, data, len, IR_I2C_TIMEOUT_MS);
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
    /* initialise samples buffer tracking structures (can't fail, do early) */
    ir_sample_cbuf = circular_buf_init(&ir_sample_cbuf_priv, ir_sample_cbuf_stor, sizeof(ir_sample_cbuf_stor));

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

/* Append one sample to the buffer, discards oldest when full. */
static void IR_SamplePush(const ir_sample_t *sample)
{
    /* cast to byte array */
    const uint8_t *raw = (const uint8_t *)sample;

    /* append to buffer */
    for (size_t i = 0; i < sizeof(*sample); i++)
        circular_buf_put(ir_sample_cbuf, raw[i]);
}

/* Take one complete sample from the buffer if available. */
static bool IR_SamplePop(ir_sample_t *out)
{
    /* cast to byte array */
    uint8_t *raw = (uint8_t *)out;

    /* ensure there is a complete sample available */
    if (circular_buf_size(ir_sample_cbuf) < sizeof(*out))
        return false;

    /* take one sample */
    for (size_t i = 0; i < sizeof(*out); i++)
        (void)circular_buf_get(ir_sample_cbuf, &raw[i]);

    return true;
}

/* Current sample timestamp counter - see the header */
uint32_t IR_TimestampNow(void)
{
    return IR_TicksTo25us(Timebase_Now());
}

/* Read one sample from the sensor into the buffer, discards oldest when full */
static int IR_ReadSample(void)
{
    ir_sample_t smp;
    int16_t presence, motion, tambient;

    /* stamp before the reads, which take three I2C1 transactions */
    smp.timestamp = IR_TimestampNow();

    /* into locals: the fields are packed, so their addresses are unaligned */
    if (IR_SENSOR_ReadPresence(&presence) != 0)
        return -1;

    if (IR_SENSOR_ReadMotion(&motion) != 0)
        return -1;

    if (IR_SENSOR_ReadTAmbient(&tambient) != 0)
        return -1;

    smp.presence = presence;
    smp.motion   = motion;
    smp.tambient = tambient;

    IR_SamplePush(&smp);

    return 1;
}

/* Take as many samples from buffer as are available and fit destination */
size_t IR_TakeSamples(ir_sample_t *dst, size_t max_count)
{
    size_t count;

    /* drain as many cached samples as available and fit dst */
    for (count = 0; count < max_count; count++) {
        if (!IR_SamplePop(&dst[count]))
            break;
    }

    return count;
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

/* IR events, as IR_SENSOR_getInt() accumulates them and as the SOM sees them
 * in the IR detail byte of Read interrupt status.
 *
 * The sensor also reports thermal shock, which is routed off the INT pin and
 * not reported; bit 2 upwards is free. */
#define IR_EVT_MOTION       (1U << 0)
#define IR_EVT_PRESENCE     (1U << 1)

/**
 * @brief Build the event byte from the sensor's status register.
 * @retval >=0  IR_EVT_* bits
 * @retval  -1  a read failed - the sensor state is unknown
 *
 * Motion and presence can be set together and are reported together. The
 * FUNC_STATUS flags are levels, re-evaluated every ODR cycle, and reading the
 * register clears none of them.
 */
static int IR_ReadEvents(void)
{
    sths34pf80_func_status_t status;
    uint8_t events = 0;

    if (sths34pf80_read_reg(&ir_sensor_ctx, STHS34PF80_FUNC_STATUS,
                            (uint8_t *)&status, 1) != 0)
        return -1;

    if (status.mot_flag)
        events |= IR_EVT_MOTION;

    if (status.pres_flag)
        events |= IR_EVT_PRESENCE;

    /* TODO: catch the bit left unmapped - status.tamb_shock_flag */

    return (int)events;
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

/* an interrupt has arrived and its reason is not read yet */
static volatile bool ir_int_pending;

/* MCU timebase tick at which that interrupt was observed */
static volatile uint32_t ir_int_tick;

/**
 * @brief Record that the INT line fired.
 *
 * Called from the EXTI handler, and once directly after
 * GPIO_EnableSensorInterrupts() - the INT pin is push-pull and level-driven,
 * so a condition already asserted when the line comes up produces no rising
 * edge at all and would otherwise never be reported.
 */
void IR_HandleInt()
{
	ir_int_tick = Timebase_Now();
	ir_int_pending = true;
}

/* process pending interrupts outside isr */
void IR_ProcessInt(void)
{
	int events;

	if (!ir_int_pending)
		return;

	/* clear before the read, so an interrupt arriving during it is kept */
	ir_int_pending = false;

	events = IR_ReadEvents();

    /* on bus error interrupt may not have been cleared, re-arm as pending */
	if (events < 0) {
		ir_int_pending = true;
		return;
	}

    /* accumulate interrupts */
	IR_INT |= (uint8_t)events;

    /* notify on a new event only, not on what is still unread in the latch */
    if (events) {
	    SomEnable();
	    somSetInt(INT_SRC_IR);
    }
}

/* samples poll interval */
#define IR_POLL_MS 1000U

/* called from main thread periodically */
void IR_Process(void)
{
    static uint32_t last_poll;
    static bool first = true;

    if (!first && (HAL_GetTick() - last_poll) < IR_POLL_MS)
        return;

    first = false;
    last_poll = HAL_GetTick();

    (void)IR_ReadSample();
}
