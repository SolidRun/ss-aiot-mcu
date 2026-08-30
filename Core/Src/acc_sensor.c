#include "acc_sensor.h"
#include "stm32u0xx_hal.h"
#include "main.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t ACC_INT;
// Static accelerometer object
static ISM330DHCX_Object_t ism330dhcx;
#define ACC_THS_DEFAULT  0x04
uint8_t acc_ths = ACC_THS_DEFAULT;


// I2C write function for driver
static int32_t ACC_I2C_Write(uint16_t handle, uint16_t Reg, uint8_t *Data, uint16_t Len) {
    return (HAL_I2C_Mem_Write(&hi2c1, 0xD5, Reg, I2C_MEMADD_SIZE_8BIT, Data, Len, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

// I2C read function for driver
static int32_t ACC_I2C_Read(uint16_t handle, uint16_t Reg, uint8_t *Data, uint16_t Len) {
    return (HAL_I2C_Mem_Read(&hi2c1, 0xD5, Reg, I2C_MEMADD_SIZE_8BIT, Data, Len, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

// Initialize the accelerometer
int ACC_Init(void) {
    ISM330DHCX_IO_t io_ctx;

    io_ctx.BusType = ISM330DHCX_I2C_BUS;
    io_ctx.Address = ISM330DHCX_I2C_ADD_L; // SA0 tied to VCC
    io_ctx.Init = NULL;
    io_ctx.DeInit = NULL;
    io_ctx.GetTick = HAL_GetTick;
    io_ctx.Delay = HAL_Delay;
    io_ctx.WriteReg = ACC_I2C_Write;
    io_ctx.ReadReg = ACC_I2C_Read;

    if (ISM330DHCX_RegisterBusIO(&ism330dhcx, &io_ctx) != ISM330DHCX_OK)
        return -1;

    if (ISM330DHCX_Init(&ism330dhcx) != ISM330DHCX_OK)
        return -1;

    // Enable accelerometer
    if (ISM330DHCX_ACC_Enable(&ism330dhcx) != ISM330DHCX_OK)
        return -1;

    // Set ODR to 104 Hz
    if (ISM330DHCX_ACC_SetOutputDataRate(&ism330dhcx, 104.0f) != ISM330DHCX_OK)
        return -1;

    // Set full scale to 2g
    if (ISM330DHCX_ACC_SetFullScale(&ism330dhcx, 2) != ISM330DHCX_OK)
        return -1;

    //Wake-up detection (movement above threshold)
    if (ISM330DHCX_ACC_Enable_Wake_Up_Detection(&ism330dhcx, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK)
        return -1;

    if (ISM330DHCX_ACC_Set_Wake_Up_Threshold(&ism330dhcx, acc_ths) != ISM330DHCX_OK)
        return -1;

    if (ISM330DHCX_ACC_Set_Wake_Up_Duration(&ism330dhcx, 0) != ISM330DHCX_OK) // instant trigger
        return -1;


    // Set latched interrupt mode
    if (ISM330DHCX_Set_Interrupt_Latch(&ism330dhcx, 1) != ISM330DHCX_OK)
        return -1;


    return 0;
}

// Read accelerometer axes
int ACC_ReadAxes(ISM330DHCX_Axes_t *axes) {
    return ISM330DHCX_ACC_GetAxes(&ism330dhcx, axes);
}

/**
 * @brief Read the wake-up source the device is reporting.
 * @retval >=0  WAKE_UP_SRC bits: 0x01 Z, 0x02 Y, 0x04 X, 0x08 WU_IA,
 *              0x20 free-fall, 0x40 sleep change
 * @retval  -1  the read failed - the device state is unknown
 *
 * Relayed as the device reports them. all_sources_get already fetches this
 * register; only wu_ia was being kept.
 *
 * SLEEP_STATE is deliberately left out. It is a state, not an event, and a
 * state bit ORed into an accumulating latch would stick there for good.
 */
int ACC_CheckWakeUp(void)
{
    ism330dhcx_all_sources_t src;

    if (ism330dhcx_all_sources_get(&ism330dhcx.Ctx, &src) != ISM330DHCX_OK) {
        return -1;
    }

    return (int)( src.wake_up_src.z_wu
              | (src.wake_up_src.y_wu             << 1)
              | (src.wake_up_src.x_wu             << 2)
              | (src.wake_up_src.wu_ia            << 3)
              | (src.wake_up_src.ff_ia            << 5)
              | (src.wake_up_src.sleep_change_ia  << 6));
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
 */
void ACC_HandleInt()
{
	int src = ACC_CheckWakeUp();

	/* abort on error */
	if (src < 0)
		return;

	/* accumulate interrupts */
	ACC_INT |= (uint8_t)src;

	/* report any interrupts to som */
	if (ACC_INT) {
		SomEnable();
		somSetInt(INT_SRC_ACCEL);
	}
}
