#include "acc_sensor.h"
#include "stm32u0xx_hal.h"
#include "main.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t ACC_INT;
// Static accelerometer object
static ISM330DHCX_Object_t ism330dhcx;
#define ACC_THS_DEFAULT  0x04
#define ACC_I2C_TIMEOUT_MS   100U
uint8_t acc_ths = ACC_THS_DEFAULT;

/* Axis cache.
 *
 * Written only by ACC_RefreshAxes() from the _6AX_INT EXTI handler, read
 * only by ACC_GetCachedAxes() from the I2C2 slave callback. Both sit at
 * NVIC priority 1, so neither can preempt the other and no lock is needed.
 * If those two priorities ever differ, this needs a guard.
 *
 * Age is reported, never used to withhold the sample. A sample older than
 * this is still the best answer available and the master gets it; the
 * status byte just says not to treat it as current. Age matters because
 * the cache can fall behind reality silently: the wake-up detector works
 * on the high-passed signal, so a slow tilt changes the orientation
 * without raising an interrupt, and a failed refresh leaves the previous
 * sample in place - in both cases the age is the only symptom. */
#define ACC_CACHE_STALE_MS  1000U

static volatile int16_t  acc_cache[3];
static volatile int16_t  acc_temp_c100;
static volatile uint32_t acc_cache_tick;
static volatile bool     acc_cache_valid = false;


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
 * The step number matters because this function stops at the first
 * failure, and the order is not arbitrary: wake-up detection is configured
 * before free-fall and tilt. A failure at step 9 therefore leaves a fully
 * working motion detector and no free-fall, no tilt and no interrupt
 * latching - which presents as "those two features are broken" rather than
 * "init stopped early".
 *
 * Both callers used to discard this value, so a partial init was invisible.
 * Sensor_Accel_Config() now passes it back as the command's status byte,
 * which makes 0x13 0x03 a way to ask the board whether its sensor came up.
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

    if (ISM330DHCX_RegisterBusIO(&ism330dhcx, &io_ctx) != ISM330DHCX_OK)
        return 1;

    if (ISM330DHCX_Init(&ism330dhcx) != ISM330DHCX_OK)
        return 2;

    // Enable accelerometer
    if (ISM330DHCX_ACC_Enable(&ism330dhcx) != ISM330DHCX_OK)
        return 3;

    /* Nominally 104 Hz - but every ISM330DHCX_ACC_Enable_*_Detection call
     * below opens by setting the ODR to 416 Hz itself, so 416 is what the
     * device actually runs at. Left in place because the wake-up threshold
     * in use was tuned against 416; changing the rate is a separate
     * decision, not a side effect of adding events. */
    if (ISM330DHCX_ACC_SetOutputDataRate(&ism330dhcx, 104.0f) != ISM330DHCX_OK)
        return 4;

    // Set full scale to 2g
    if (ISM330DHCX_ACC_SetFullScale(&ism330dhcx, 2) != ISM330DHCX_OK)
        return 5;

    //Wake-up detection (movement above threshold)
    if (ISM330DHCX_ACC_Enable_Wake_Up_Detection(&ism330dhcx, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK)
        return 6;

    if (ISM330DHCX_ACC_Set_Wake_Up_Threshold(&ism330dhcx, acc_ths) != ISM330DHCX_OK)
        return 7;

    if (ISM330DHCX_ACC_Set_Wake_Up_Duration(&ism330dhcx, 0) != ISM330DHCX_OK) // instant trigger
        return 8;

    /* Free-fall detection. ff_ia was already being reported in the wake-up
     * source byte, but nothing ever enabled the detector, so the bit could
     * never be set. */
    if (ISM330DHCX_ACC_Enable_Free_Fall_Detection(&ism330dhcx, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK)
        return 9;

    /* Duration in ODR samples, so 36 ms at 416 Hz. ST's own default is 6
     * samples - 14 ms, short enough that a downward flick of the wrist
     * can read as a fall. 15 is 2.5x stricter than that and still only a
     * 0.6 cm drop, so any real fall clears it comfortably.
     *
     * Chosen to make the event reachable by hand, deliberately: a first
     * test that cannot trigger tells us nothing about whether the config
     * is right. Tighten it once the chain is known to work. */
    if (ISM330DHCX_ACC_Set_Free_Fall_Duration(&ism330dhcx, 15) != ISM330DHCX_OK)
        return 10;

    /* Tilt detection, an embedded function. The wake-up detector works on
     * the high-passed signal, so it sees change and not position: tilt the
     * board slowly and the orientation changes with no interrupt at all.
     * This function is built for exactly that case. */
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

    // Set latched interrupt mode
    if (ISM330DHCX_Set_Interrupt_Latch(&ism330dhcx, 1) != ISM330DHCX_OK)
        return 15;


    return 0;
}

// Read accelerometer axes
int ACC_ReadAxes(ISM330DHCX_Axes_t *axes) {
    return ISM330DHCX_ACC_GetAxes(&ism330dhcx, axes);
}

/**
 * @brief Sample the three axes into the cache.
 * @retval  0  cache updated
 * @retval -1  the read failed - the cache keeps its previous contents
 *
 * GetAxesRaw rather than GetAxes: GetAxes re-reads the full-scale register
 * on every call to work out the sensitivity, which is a second bus
 * transaction and a second thing that can fail, for a value ACC_Init()
 * wrote itself and nothing changes afterwards. The conversion is done here
 * in integers instead - this part has no FPU.
 */
int ACC_RefreshAxes(void)
{
    ISM330DHCX_AxesRaw_t raw;
    int16_t              traw;

    if (ISM330DHCX_ACC_GetAxesRaw(&ism330dhcx, &raw) != ISM330DHCX_OK)
        return -1;

    if (ism330dhcx_temperature_raw_get(&ism330dhcx.Ctx, &traw) != ISM330DHCX_OK)
        return -1;

    /* 0.061 mg/LSB at +-2g. raw is +-32767, so raw * 61 stays well inside
     * int32_t. At +-2g the result is +-2000 mg and fits int16_t; a larger
     * full scale would need this widened. */
    acc_cache[0] = (int16_t)(((int32_t)raw.x * 61) / 1000);
    acc_cache[1] = (int16_t)(((int32_t)raw.y * 61) / 1000);
    acc_cache[2] = (int16_t)(((int32_t)raw.z * 61) / 1000);

    /* 256 LSB/degC, with 0 LSB meaning 25 degC. Reported in hundredths of
     * a degree so the master needs no scaling table. */
    acc_temp_c100 = (int16_t)(2500 + (((int32_t)traw * 100) / 256));

    acc_cache_tick  = HAL_GetTick();
    acc_cache_valid = true;
    return 0;
}

/**
 * @brief Hand the last sample to the protocol layer.
 * @retval 0  xyz holds a sample no older than ACC_CACHE_STALE_MS
 * @retval 1  nothing has ever been sampled - xyz is untouched
 * @retval 2  xyz holds a real sample, but older than ACC_CACHE_STALE_MS
 *
 * The return value is the protocol status byte directly.
 *
 * Deliberately does not touch the bus. It is called from the I2C2 slave
 * callback, and a bus read there could be preempted by - or preempt - the
 * sensor EXTI handlers, which are the other users of I2C1. It raises the
 * _6AX_INT line in software instead; that handler runs as soon as this
 * callback returns, at the same priority, so it can never collide with a
 * transfer already on the bus.
 *
 * The refresh therefore lands after the response has been frozen into
 * txBuffer, and this call always serves the sample the *previous* read
 * asked for. Waiting for a fresh one is not possible from here, so the
 * sample is taken ahead of the next read instead and a master polling at
 * any rate gets a value every time.
 *
 * Which is why status 2 is normal, not exceptional: for a master polling
 * slower than a second, the sample it receives was taken one poll
 * interval ago and is legitimately stale. The value is still the best
 * answer available - status 2 says how much to trust it, not to drop it.
 */
int ACC_GetCachedAxes(int16_t *xyz, int16_t *temp_c100)
{
    __HAL_GPIO_EXTI_GENERATE_SWIT(_6AX_INT_Pin);

    if (!acc_cache_valid)
        return 1;

    xyz[0] = acc_cache[0];
    xyz[1] = acc_cache[1];
    xyz[2] = acc_cache[2];
    *temp_c100 = acc_temp_c100;

    return ((HAL_GetTick() - acc_cache_tick) > ACC_CACHE_STALE_MS) ? 2 : 0;
}

/**
 * @brief Read the event sources the device is reporting.
 * @retval >=0  0x01 Z, 0x02 Y, 0x04 X, 0x08 WU_IA, 0x10 tilt,
 *              0x20 free-fall, 0x40 sleep change
 * @retval  -1  a read failed - the device state is unknown
 *
 * Relayed as the device reports them. all_sources_get already fetches
 * WAKE_UP_SRC; only wu_ia was being kept.
 *
 * Two registers, not one: tilt is an embedded function and lands in
 * EMB_FUNC_STATUS_MAINPAGE instead. Both are read directly rather than
 * through ALL_INT_SRC, which is what all_sources_get did - reading that
 * register first cleared WU_IA before WAKE_UP_SRC could be read.
 *
 * SLEEP_STATE is deliberately left out. It is a state, not an event, and a
 * state bit ORed into an accumulating latch would stick there for good.
 * Which does leave sleep_change_ia saying that the state flipped without
 * saying to what - worth revisiting.
 */
int ACC_CheckWakeUp(void)
{
    ism330dhcx_wake_up_src_t                src;
    ism330dhcx_emb_func_status_mainpage_t   emb;

    if (ism330dhcx_read_reg(&ism330dhcx.Ctx, ISM330DHCX_WAKE_UP_SRC,
                            (uint8_t *)&src, 1) != ISM330DHCX_OK) {
        return -1;
    }

    if (ism330dhcx_read_reg(&ism330dhcx.Ctx, ISM330DHCX_EMB_FUNC_STATUS_MAINPAGE,
                            (uint8_t *)&emb, 1) != ISM330DHCX_OK) {
        return -1;
    }

    return (int)( src.z_wu
              | (src.y_wu            << 1)
              | (src.x_wu            << 2)
              | (src.wu_ia           << 3)
              | (emb.is_tilt         << 4)
              | (src.ff_ia           << 5)
              | (src.sleep_change_ia << 6));
}

int ACC_getInt()
{
	return ACC_INT;
}

void ACC_clearInt()
{
	ACC_INT = 0;
}

/**
 * @brief Sample the wake-up source and notify the SOM if it fired.
 *
 * Called from the EXTI handler, and once directly after
 * GPIO_EnableSensorInterrupts() - the wake-up is latched in the device, so
 * one that fired before the line came up produces no further edge.
 *
 * Also reached with the line raised in software by ACC_GetCachedAxes(),
 * which wants a sample and no notification. That case needs no flag: the
 * device is in latched mode, WAKE_UP_SRC clears on read, so a software
 * edge reads zero and falls through the notify below.
 */
void ACC_HandleInt()
{
	int src = ACC_CheckWakeUp();

	/* sample the axes on every interrupt, real or software-generated */
	(void)ACC_RefreshAxes();

	/* abort on error */
	if (src < 0)
		return;

	/* accumulate interrupts */
	ACC_INT |= (uint8_t)src;

	/* report only a new event. Testing the accumulated latch instead would
	 * re-notify on every later interrupt until the SOM read it, and would
	 * fire on a software edge raised only to refresh the axes. */
	if (src) {
		SomEnable();
		somSetInt(INT_SRC_ACCEL);
	}
}
