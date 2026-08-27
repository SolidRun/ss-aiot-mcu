#include "acc_sensor.h"
#include "stm32u0xx_hal.h"
#include "main.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1;
extern bool ACC_INT;
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

// Function to check wake-up source
bool ACC_CheckWakeUp(void) {
    ism330dhcx_all_sources_t src;
    // Read all sources from sensor
    if (ism330dhcx_all_sources_get(&ism330dhcx.Ctx, &src) == ISM330DHCX_OK) {
        // Check if wake-up interrupt occurred
        if (src.all_int_src.wu_ia) {
            // Wake-up detected
        	return 1;
        } else {
            // No wake-up
        	return 0;
        }
    }
    return 0;
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
	ACC_INT = ACC_CheckWakeUp();
	if (ACC_INT) {
		SomEnable();
		somSetInt();
	}
}
