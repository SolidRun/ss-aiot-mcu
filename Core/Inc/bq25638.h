#ifndef BQ25638_H
#define BQ25638_H

#include "stm32u0xx_hal.h"
#include "main.h"

// I2C address of BQ25638
#define BQ25638_I2C_ADDR   (0x6B << 1)  // 7-bit shifted left for HAL

// BQ25638 Register Addresses
#define BQ25638_REG_CHG_CURRENT_LIMIT 0x02
#define BQ25638_REG_CHG_VOLTAGE_LIMIT 0x04
#define BQ25638_REG_INPUT_CURRENT_LIMIT 0x06
#define BQ25638_REG_INPUT_VOLTAGE_LIMIT 0x08
#define BQ25638_REG_MIN_SYS_VOLTAGE 0x0E
#define BQ25638_REG_PRECHARGE_CTRL 0x10
#define BQ25638_REG_TERM_CTRL 0x12
#define BQ25638_REG_CHG_TIMER_CTRL 0x14
#define BQ25638_REG_CHG_CTRL_0 0x15
#define BQ25638_REG_CHG_CTRL_1 0x16
#define BQ25638_REG_CHG_CTRL_2 0x17
#define BQ25638_REG_CHG_CTRL_3 0x18
#define BQ25638_REG_CHG_CTRL_4 0x19
#define BQ25638_REG_CHG_CTRL_5 0x1A
#define BQ25638_REG_CHG_STATUS_0 0x20
#define BQ25638_REG_CHG_STATUS_1 0x21
#define BQ25638_REG_FAULT_STATUS 0x22
#define BQ25638_REG_CHG_FLAG_0 0x23
#define BQ25638_REG_CHG_FLAG_1 0x24
#define BQ25638_REG_FAULT_FLAG 0x25
#define BQ25638_REG_CHG_MASK_0 0x26
#define BQ25638_REG_CHG_MASK_1 0x27
#define BQ25638_REG_FAULT_MASK 0x28
#define BQ25638_REG_ADC_CTRL  0x2B
#define BQ25638_REG_NTC_CTRL0 0x1C
#define BQ25638_REG_PART_INFO 0x3F
#define BQ25638_REG_VBAT_ADC 0x35

// GPIO definitions for board connections
#define BQ25638_BATT_CE_PORT   BATT_CE_GPIO_Port
#define BQ25638_BATT_CE_PIN    BATT_CE_Pin

#define BQ25638_BAT_QON_PORT   BATT_QON_GPIO_Port
#define BQ25638_BAT_QON_PIN    BATT_QON_Pin

#define BQ25638_STAT_PORT      BATT_STAT_GPIO_Port
#define BQ25638_STAT_PIN       BATT_STAT_Pin

#define BQ25638_INT_PORT       BATT_INT_GPIO_Port
#define BQ25638_INT_PIN        BATT_INT_Pin

#define BQ25638_PG_PORT        BATT_PG_GPIO_Port
#define BQ25638_PG_PIN         BATT_PG_Pin




    //Charge Status:
    //0 = Not Charging
    //1 = Trickle Charge
    //2 = Pre-charge
    //3 = Fast Charge (CC)
    //4 = Taper Charge (CV)
    //6 = Top-off Timer Active Charging
    //7 = Charge Termination Done
typedef struct {
    uint8_t power_source;   // 0=Not powered from VBUS; 4=Unknown adaptor; 7=In boost OTG;
    uint8_t charge_status;  // 0=Not Charging;  2=Pre-charge; 3=Fast Charge; 7=Charge Termination Done;
    uint8_t battery_soc;    // Battery State of Charge (%)
} BQ25638_Status_t;

//I2C Basic Functions
HAL_StatusTypeDef BQ25638_WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef BQ25638_ReadReg(uint8_t reg, uint8_t *data);

//Charging Control
HAL_StatusTypeDef BQ25638_EnableCharging();
HAL_StatusTypeDef BQ25638_SetChargeCurrent(uint16_t mA);
HAL_StatusTypeDef BQ25638_SetChargeVoltage(uint16_t mV);
HAL_StatusTypeDef BQ25638_SetTerminationCurrent(uint16_t mA);

//Status Reading
BQ25638_Status_t BQ25638_GetStatus(void);
uint8_t BQ25638_GetChargeStatus(void);
uint8_t BQ25638_GetFault(void);

//=GPIO Helpers
void BQ25638_SetCE(uint8_t value);
uint8_t BQ25638_ReadQON(void);
uint8_t BQ25638_ReadSTAT(void);
uint8_t BQ25638_ReadINT(void);
uint8_t BQ25638_ReadPG(void);

//Interrupt Handling
// Callback function prototype
void BQ25638_INT_Callback(void);


//Initialization
HAL_StatusTypeDef BQ25638_Init(void);

#endif /* BQ25638_H */
