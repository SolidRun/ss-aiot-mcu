#include "acc_sensor.h"
#include "stm32u0xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include <string.h>
#include "circular_buffer.h"
#include "timebase.h"

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t ACC_INT;
// Static accelerometer object
static ISM330DHCX_Object_t ism330dhcx;
#define ACC_THS_DEFAULT  0x04
#define ACC_I2C_TIMEOUT_MS   100U
uint8_t acc_ths = ACC_THS_DEFAULT;

/* Output data rate, base for batch rates. */
// TODO: reduce to 52Hz
#define ACC_ODR_HZ 416U

/* accelerometer batch rate must follow ODR */
#if ACC_ODR_HZ == 416U
#define ACC_BDR_SETTING  ISM330DHCX_XL_BATCHED_AT_417Hz
#elif ACC_ODR_HZ == 52U
#define ACC_BDR_SETTING  ISM330DHCX_XL_BATCHED_AT_52Hz
#else
#error "no ISM330DHCX_XL_BATCHED_AT_* setting for this ACC_ODR_HZ"
#endif

/* minimum duration for free-fall detection */
#define ACC_FF_DURATION_MS 36U
/*
 * Free-fall duration is programmed in number of samples reltive to ODR.
 * Calculate sample count from intended duration:
 * samples = ms * ODR / 1000, the +500 rounds to nearest instead of truncating.
 * Result must stay below 32, FF_DUR is a 5-bit field. 15 samples at 416Hz, 2 at 52Hz.
 */
#define ACC_FF_DURATION (((ACC_FF_DURATION_MS * ACC_ODR_HZ) + 500U) / 1000U)

/*
 * Samples the wake-up threshold must be exceeded for before the event fires.
 * WAKE_DUR is a 2-bit field, 1 LSB = 1 ODR period, so 0 fires on the first
 * sample above the threshold - the most sensitive setting there is.
 */
#define ACC_WAKE_UP_DURATION 0U

/* The device timestamp counts 25us per LSB, so 40000 of them to the second.
 * ACC_TS_LSB_PER_SAMPLE stamps the samples that fall between two batched
 * timestamps - 96 at 416Hz, 769 at 52Hz. */
#define ACC_TS_LSB_PER_SEC     40000U
#define ACC_TS_LSB_PER_SAMPLE  (ACC_TS_LSB_PER_SEC / ACC_ODR_HZ)

/* ACC_TicksTo25us() below hardcodes the ratio of these two constants as
 * 625/64 instead of referencing them, so pin the contract. */
_Static_assert(ACC_TS_LSB_PER_SEC * 64U == TIMEBASE_HZ * 625U, "625/64 is no longer ACC_TS_LSB_PER_SEC / TIMEBASE_HZ");

/* Restate an MCU tick count in 25us units, scaling by 625/64 modulo 2^32.
 * Truncates by at most 1 LSB. */
static uint32_t ACC_TicksTo25us(uint32_t ticks)
{
    return ((ticks >> 6) * 625U) + (((ticks & 0x3FU) * 625U) >> 6);
}

/* Allocate circular buffer for samples. Enough for 250ms at ODR 416Hz or 2s at ODR 52Hz */
#define ACC_MOTIONSAMPLE_BUF_SIZE 104U
static cbuf_handle_t acc_motionsample_cbuf;
static struct circular_buf_t acc_motionsample_cbuf_priv;
static uint8_t acc_motionsample_cbuf_stor[ACC_MOTIONSAMPLE_BUF_SIZE * sizeof(acc_motionsample_t)];

/* Cache latest die temperature and its timestamp */
static acc_tempsample_t acc_tempsample;
static bool acc_tempsample_valid;

/* Append one sample to the buffer, discards oldest when full. */
static void ACC_MotionSamplePush(const acc_motionsample_t *sample)
{
    /* cast to byte array */
    const uint8_t *raw = (const uint8_t *)sample;

    /* append to buffer */
    for (size_t i = 0; i < sizeof(*sample); i++)
        circular_buf_put(acc_motionsample_cbuf, raw[i]);
}

/* Take one complete sample from the buffer if available. */
static bool ACC_MotionSamplePop(acc_motionsample_t *out)
{
    /* cast to byte array */
    uint8_t *raw = (uint8_t *)out;

    /* ensure there is a complete sample available */
    if (circular_buf_size(acc_motionsample_cbuf) < sizeof(*out))
        return false;

    /* take one sample */
    for (size_t i = 0; i < sizeof(*out); i++)
        (void)circular_buf_get(acc_motionsample_cbuf, &raw[i]);

    return true;
}

/* Current sample timestamp counter - see the header */
uint32_t ACC_TimestampNow(void)
{
    return ACC_TicksTo25us(Timebase_Now());
}

/* Take as many samples from buffer as are available and fit destination */
size_t ACC_TakeMotionSamples(acc_motionsample_t *dst, size_t max_count)
{
    size_t count;

    /* drain as many cached samples as available and fit dst */
    for (count = 0; count < max_count; count++) {
        if (!ACC_MotionSamplePop(&dst[count]))
            break;
    }

    return count;
}

/* Take as many samples from buffer as are available and fit destination */
size_t ACC_TakeTempSamples(acc_tempsample_t *dst, size_t max_count)
{
    if (!acc_tempsample_valid)
        /* no samples available */
        return 0;

    if (max_count == 0)
        return 0;

    /* there is only one sample, and handing it out consumes it */
    *dst = acc_tempsample;
    acc_tempsample_valid = false;

    return 1;
}

// I2C write function for driver
static int32_t ACC_I2C_Write(uint16_t handle, uint16_t Reg, uint8_t *Data, uint16_t Len) {
    return (HAL_I2C_Mem_Write(&hi2c1, 0xD5, Reg, I2C_MEMADD_SIZE_8BIT, Data, Len, ACC_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

// I2C read function for driver
static int32_t ACC_I2C_Read(uint16_t handle, uint16_t Reg, uint8_t *Data, uint16_t Len) {
    return (HAL_I2C_Mem_Read(&hi2c1, 0xD5, Reg, I2C_MEMADD_SIZE_8BIT, Data, Len, ACC_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int32_t ACC_GetTick(void)
{
    return (int32_t)HAL_GetTick();
}

/**
 * @brief Bring up the accelerometer and every event it should report.
 * @retval 0   every step succeeded
 * @retval >0  the number of the step that failed, counting the returns below
 *
 * Stops at the first failure, so the steps after it are not applied. Wake-up
 * detection is configured before free-fall and tilt.
 */
int ACC_Init(void) {
    ISM330DHCX_IO_t io_ctx;

    io_ctx.BusType = ISM330DHCX_I2C_BUS;
    io_ctx.Address = ISM330DHCX_I2C_ADD_L; // SA0 tied to VCC
    io_ctx.Init = NULL;
    io_ctx.DeInit = NULL;
    io_ctx.GetTick = ACC_GetTick;
    io_ctx.Delay = HAL_Delay;
    io_ctx.WriteReg = ACC_I2C_Write;
    io_ctx.ReadReg = ACC_I2C_Read;

    /* initialise samples buffer tracking structures (can't fail, do early) */
    acc_motionsample_cbuf = circular_buf_init(&acc_motionsample_cbuf_priv, acc_motionsample_cbuf_stor,
                                 sizeof(acc_motionsample_cbuf_stor));

    if (ISM330DHCX_RegisterBusIO(&ism330dhcx, &io_ctx) != ISM330DHCX_OK)
        return 1;

    if (ISM330DHCX_Init(&ism330dhcx) != ISM330DHCX_OK)
        return 2;

    // Enable accelerometer
    if (ISM330DHCX_ACC_Enable(&ism330dhcx) != ISM330DHCX_OK)
        return 3;

    // TODO: ism330dhcx_xl_power_mode_set(LOW_NORMAL_POWER_MD) for XL_HM_MODE=1 - 360uA -> 32uA
    /* Every ISM330DHCX_ACC_Enable_*_Detection call below sets the ODR to
     * 416 Hz itself, so this only holds until the first of them. */
    if (ISM330DHCX_ACC_SetOutputDataRate(&ism330dhcx, (float)ACC_ODR_HZ) != ISM330DHCX_OK)
        return 4;

    // Set full scale to 2g
    if (ISM330DHCX_ACC_SetFullScale(&ism330dhcx, 2) != ISM330DHCX_OK)
        return 5;

    //Wake-up detection (movement above threshold)
    if (ISM330DHCX_ACC_Enable_Wake_Up_Detection(&ism330dhcx, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK)
        return 6;

    // TODO: retune after changing ODR to 52Hz
    if (ISM330DHCX_ACC_Set_Wake_Up_Threshold(&ism330dhcx, acc_ths) != ISM330DHCX_OK)
        return 7;

    /*
     * enable free-fall detection
     *
     * Notes:
     * - sets minimum free-fall duration to 6 samples
     * - sets wake-up duration to 0
     */
    if (ISM330DHCX_ACC_Enable_Free_Fall_Detection(&ism330dhcx, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK)
        return 8;

    /* set intended minimum free-fall duration */
    if (ISM330DHCX_ACC_Set_Free_Fall_Duration(&ism330dhcx, ACC_FF_DURATION) != ISM330DHCX_OK)
        return 9;

    /* set intended wake-up duration */
    if (ISM330DHCX_ACC_Set_Wake_Up_Duration(&ism330dhcx, ACC_WAKE_UP_DURATION) != ISM330DHCX_OK)
        return 10;

    /* Tilt detection, an embedded function. The wake-up detector works on
     * the high-passed signal, so it sees change and not position: tilt the
     * board slowly and the orientation changes with no interrupt at all.
     * This function is built for exactly that case. */
    // TODO: verify tilt after changing ODR to 52Hz
    if (ism330dhcx_tilt_sens_set(&ism330dhcx.Ctx, PROPERTY_ENABLE) != ISM330DHCX_OK)
        return 11;

    /* An embedded function needs an init pulse as well as an enable - the
     * algorithm does not start on tilt_en alone. ST's driver exposes no
     * setter for EMB_FUNC_INIT_A, so it is written by hand. */
    {
        ism330dhcx_emb_func_init_a_t init_a;
        int32_t r;

        r = ism330dhcx_mem_bank_set(&ism330dhcx.Ctx, ISM330DHCX_EMBEDDED_FUNC_BANK);

        if (r == ISM330DHCX_OK) {
            r = ism330dhcx_read_reg(&ism330dhcx.Ctx, ISM330DHCX_EMB_FUNC_INIT_A,
                                    (uint8_t *)&init_a, 1);
        }

        if (r == ISM330DHCX_OK) {
            init_a.tilt_init = PROPERTY_ENABLE;
            r = ism330dhcx_write_reg(&ism330dhcx.Ctx, ISM330DHCX_EMB_FUNC_INIT_A,
                                     (uint8_t *)&init_a, 1);
        }

        /* Switch back whatever happened above. Returning early from inside
         * the embedded bank would leave every later register access
         * pointing at the wrong page, silently. */
        if (ism330dhcx_mem_bank_set(&ism330dhcx.Ctx, ISM330DHCX_USER_BANK) != ISM330DHCX_OK)
            r = -1;

        if (r != ISM330DHCX_OK)
            return 12;
    }

    {
        ism330dhcx_pin_int1_route_t route;

        if (ism330dhcx_pin_int1_route_get(&ism330dhcx.Ctx, &route) != ISM330DHCX_OK)
            return 13;

        route.emb_func_int1.int1_tilt = PROPERTY_ENABLE;

        if (ism330dhcx_pin_int1_route_set(&ism330dhcx.Ctx, &route) != ISM330DHCX_OK)
            return 14;
    }

    /*Latch every interrupt, base functions and embedded alike, so an event
     * holds until it is read instead of self-clearing under the handler.
     */
    if (ism330dhcx_int_notification_set(&ism330dhcx.Ctx, ISM330DHCX_ALL_INT_LATCHED) != ISM330DHCX_OK)
        return 15;

    // TODO: re-assert ACC_ODR_HZ here - Enable_Wake_Up/Free_Fall_Detection each forced 416Hz

    /* enable timestamping on samples so movement can be reconstructed */
    /* nominal resolution 25us, INTERNAL_FREQ_FINE gives real figure */
    if (ism330dhcx_timestamp_set(&ism330dhcx.Ctx, PROPERTY_ENABLE) != ISM330DHCX_OK)
        return 16;

    /* gyroscope not enabled, disable batching */
    if (ism330dhcx_fifo_gy_batch_set(&ism330dhcx.Ctx, ISM330DHCX_GY_NOT_BATCHED) != ISM330DHCX_OK)
        return 17;

    /* set accelerometer batch rate (must match ODR) */
    if (ism330dhcx_fifo_xl_batch_set(&ism330dhcx.Ctx, ACC_BDR_SETTING) != ISM330DHCX_OK)
        return 18;

    /* batch temperature readings at slowest (1.6Hz) rate */
    if (ism330dhcx_fifo_temp_batch_set(&ism330dhcx.Ctx, ISM330DHCX_TEMP_BATCHED_AT_1Hz6) != ISM330DHCX_OK)
        return 19;

    /* generate timestamp every 8 samples, intermediate samples estimated by +=n/ODR */
    if (ism330dhcx_fifo_timestamp_decimation_set(&ism330dhcx.Ctx, ISM330DHCX_DEC_8) != ISM330DHCX_OK)
        return 20;

    /* temporarily enable bypass mode to clear fifo */
    if (ism330dhcx_fifo_mode_set(&ism330dhcx.Ctx, ISM330DHCX_BYPASS_MODE) != ISM330DHCX_OK)
        return 21;

    /* set fifo to continuous (stream) mode, discarding old samples automatically when full */
    if (ism330dhcx_fifo_mode_set(&ism330dhcx.Ctx, ISM330DHCX_STREAM_MODE) != ISM330DHCX_OK)
        return 22;

    return 0;
}

// Read accelerometer axes
int ACC_ReadAxes(ISM330DHCX_Axes_t *axes) {
    return ISM330DHCX_ACC_GetAxes(&ism330dhcx, axes);
}

/* Accelerometer events, as ACC_getInt() accumulates them and as the SOM sees
 * them in the accelerometer detail byte of Read interrupt status.
 *
 * The device also reports the wake-up axes separately, a wake-up summary and
 * activity-state changes. None of those is reported; bit 3 upwards is free. */
#define ACC_EVT_MOTION      (1U << 0)
#define ACC_EVT_TILT        (1U << 1)
#define ACC_EVT_FREEFALL    (1U << 2)

/* events that should capture motion samples */
#define ACC_EVT_CAPTURE (ACC_EVT_MOTION | ACC_EVT_FREEFALL)

/**
 * @brief Build the event byte from the device's status registers.
 * @retval >=0  ACC_EVT_* bits
 * @retval  -1  a read failed - the device state is unknown
 *
 * Two registers are read: the wake-up axes from WAKE_UP_SRC, tilt from
 * EMB_FUNC_STATUS_MAINPAGE. Neither goes through ALL_INT_SRC, reading which
 * clears WU_IA.
 */
static int ACC_ReadEvents(void)
{
    ism330dhcx_wake_up_src_t src;
    ism330dhcx_emb_func_status_mainpage_t emb;
    uint8_t events = 0;

    if (ism330dhcx_read_reg(&ism330dhcx.Ctx, ISM330DHCX_WAKE_UP_SRC,
                            (uint8_t *)&src, 1) != ISM330DHCX_OK)
        return -1;

    if (ism330dhcx_read_reg(&ism330dhcx.Ctx, ISM330DHCX_EMB_FUNC_STATUS_MAINPAGE,
                            (uint8_t *)&emb, 1) != ISM330DHCX_OK)
        return -1;

    /* any axis over the wake-up threshold, without saying which */
    if (src.x_wu || src.y_wu || src.z_wu)
        events |= ACC_EVT_MOTION;

    if (emb.is_tilt)
        events |= ACC_EVT_TILT;

    if (src.ff_ia)
        events |= ACC_EVT_FREEFALL;

    /* TODO: catch unmapped events */

    return (int)events;
}

int ACC_getInt()
{
	return ACC_INT;
}

void ACC_clearInt()
{
	ACC_INT = 0;
}

/* Drain device FIFO into RAM buffer, discarding old samples when full. */
int ACC_DrainFifo(void)
{
    /* Timestamp reconstruction state. Carried across drains: an anchor stays
     * valid until the next one arrives. */
    static uint32_t acc_ts_anchor;
    static uint16_t acc_ts_offset;
    static bool acc_ts_valid;
    uint16_t level;
    uint16_t i;
    int pushed = 0;
    uint32_t drain_tick;
    uint32_t device_now;
    uint32_t ts_offset;

    /* read the MCU timebase tick counter */
    drain_tick = Timebase_Now();

    /* read the ISM330DHCX timestamp counter, 0.5ms later at 100kHz */
    if (ism330dhcx_timestamp_raw_get(&ism330dhcx.Ctx, &device_now) != ISM330DHCX_OK)
        return -1;

    /* offset from the ISM330DHCX timebase to the MCU timebase, 25us units */
    ts_offset = ACC_TicksTo25us(drain_tick) - device_now;

    if (ism330dhcx_fifo_data_level_get(&ism330dhcx.Ctx, &level) != ISM330DHCX_OK)
        return -1;

    for (i = 0; i < level; i++) {
        ism330dhcx_fifo_tag_t tag;
        uint8_t raw[6];
        acc_motionsample_t smp = {0};
        uint32_t stamp;

        if (ism330dhcx_fifo_sensor_tag_get(&ism330dhcx.Ctx, &tag) != ISM330DHCX_OK) {
            /* we may already have read some bytes though, abort without error */
            break;
        }

        if (ism330dhcx_fifo_out_raw_get(&ism330dhcx.Ctx, raw) != ISM330DHCX_OK) {
            /* we may already have read some bytes though, abort without error */
            break;
        }

        switch (tag) {
        case ISM330DHCX_TIMESTAMP_TAG:
            /* 32-bit counter in the low four data bytes, little-endian like
             * every other multi-byte value on this part. */
            acc_ts_anchor = ((uint32_t)raw[3] << 24) | ((uint32_t)raw[2] << 16)
                          | ((uint32_t)raw[1] << 8)  | (uint32_t)raw[0];
            acc_ts_offset = 0;
            acc_ts_valid  = true;
            break;

        case ISM330DHCX_XL_NC_TAG:
            if (!acc_ts_valid)
                /* no anchor yet - see above */
                break;

            /* interpolate from the anchor, then shift onto the MCU timebase */
            stamp = acc_ts_anchor;
            stamp += (uint32_t)acc_ts_offset * ACC_TS_LSB_PER_SAMPLE;
            stamp += ts_offset;

            smp.timestamp = stamp;
            smp.x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
            smp.y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
            smp.z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

            acc_ts_offset++;
            ACC_MotionSamplePush(&smp);
            pushed++;
            break;

        case ISM330DHCX_TEMPERATURE_TAG:
            if (!acc_ts_valid)
                /* no anchor yet - see above */
                break;

            /* interpolate from the anchor, then shift onto the MCU timebase */
            stamp = acc_ts_anchor;
            stamp += (uint32_t)acc_ts_offset * ACC_TS_LSB_PER_SAMPLE;
            stamp += ts_offset;

            acc_tempsample.temp = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
            acc_tempsample.timestamp = stamp;
            acc_tempsample_valid = true;

            /* a temperature reading shares a sample slot rather than adding one, don't increment acc_ts_offset */
            break;

        default:
            /* unhandled, discard */
            break;
        }
    }

    return pushed;
}

/**
 * @brief Read the events and notify the SOM if any fired.
 *
 * Called from the EXTI handler, and once directly after
 * GPIO_EnableSensorInterrupts() - the events are latched in the device, so one
 * that fired before the line came up produces no further edge.
 */
void ACC_HandleInt()
{
	int events = ACC_ReadEvents();

	/* abort on error */
	if (events < 0)
		return;

	/* accumulate interrupts */
	ACC_INT |= (uint8_t)events;

	/* notify on a new event only, not on what is still unread in the latch */
	if (events) {
		/* Capture before notifying, so the samples are in RAM by the time the
		 * SOM asks for them. */
		if (events & ACC_EVT_CAPTURE)
			(void)ACC_DrainFifo();

		SomEnable();
		somSetInt(INT_SRC_ACCEL);
	}
}
