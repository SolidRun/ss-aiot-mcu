#include "bq25638.h"

extern I2C_HandleTypeDef hi2c1; // I2C handle from CubeMX

// I2C Basic Functions

// Write one byte to a register
HAL_StatusTypeDef BQ25638_WriteReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, BQ25638_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// Read one byte from a register
HAL_StatusTypeDef BQ25638_ReadReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(&hi2c1, BQ25638_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

// Read 16-bit register (big-endian)
HAL_StatusTypeDef BQ25638_ReadReg16(uint8_t reg, uint16_t *data) {
    uint8_t buf[2];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, BQ25638_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if(ret == HAL_OK) {
        *data = (buf[1] << 8) | buf[0];
    }
    return ret;
}

//Charging Control

// Enable or disable charging
HAL_StatusTypeDef BQ25638_EnableCharging() {
    uint8_t val;
    if(BQ25638_ReadReg(BQ25638_REG_CHG_CTRL_1, &val) != HAL_OK) return HAL_ERROR;
    val |= (1 << 5);   // CHG_ENABLE bit
    val &= ~(0x03); //WATCHDOG disable
    return BQ25638_WriteReg(BQ25638_REG_CHG_CTRL_1, val);
}


HAL_StatusTypeDef BQ25638_SetTsIgnore(uint8_t value) {
    uint8_t val;
    if(BQ25638_ReadReg(BQ25638_REG_NTC_CTRL0, &val) != HAL_OK) return HAL_ERROR;
    if(value) val |= (1 << 7);   // TS_IGNORE bit
    else       val &= ~(1 << 7);
    return BQ25638_WriteReg(BQ25638_REG_NTC_CTRL0, val);
}

HAL_StatusTypeDef BQ25638_EnableAdc(uint8_t enable){
    uint8_t val;
    if (BQ25638_ReadReg(BQ25638_REG_ADC_CTRL, &val) != HAL_OK)
        return HAL_ERROR;
    if (enable) val |= (1 << 7);    // Set EN_ADC bit (bit 7) to enable ADC
    else val &= ~(1 << 7);   // Clear EN_ADC bit (bit 7) to disable ADC
    return BQ25638_WriteReg(BQ25638_REG_ADC_CTRL, val);
}

// Set charge current in mA (512-3072mA, step 64mA)
HAL_StatusTypeDef BQ25638_SetChargeCurrent(uint16_t mA) {
    uint8_t val;
    if(mA < 512) mA = 512;
    if(mA > 3072) mA = 3072;
    val = (mA - 512) / 64; // 64mA per step
    return BQ25638_WriteReg(BQ25638_REG_CHG_CURRENT_LIMIT, val);
}

// Set charge voltage in mV (3840-4608mV, step 16mV)
HAL_StatusTypeDef BQ25638_SetChargeVoltage(uint16_t mV) {
    uint8_t val;
    if(mV < 3840) mV = 3840;
    if(mV > 4608) mV = 4608;
    val = (mV - 3840) / 16; // 16mV per step
    return BQ25638_WriteReg(BQ25638_REG_CHG_VOLTAGE_LIMIT, val);
}

// Set termination current in mA (64-512mA, step 64mA)
HAL_StatusTypeDef BQ25638_SetTerminationCurrent(uint16_t mA) {
    uint8_t val;
    if(mA < 64) mA = 64;
    if(mA > 512) mA = 512;
    val = (mA - 64) / 64;
    return BQ25638_WriteReg(BQ25638_REG_TERM_CTRL, val);
}


// Status Reading
BQ25638_Status_t BQ25638_GetStatus() {

	BQ25638_Status_t status = {0};
    uint8_t chg_status1 = 0;
    uint16_t raw;

    //Read battery status
    if (BQ25638_ReadReg(BQ25638_REG_CHG_STATUS_1, &chg_status1) == HAL_OK) {
    	status.power_source = chg_status1 & 0x07;
    	status.charge_status = ( chg_status1 >> 3) & 0x07;

    }else{
    	status.power_source = 0xFF;
    	status.charge_status = 0xFF;
    }

    //Read battery voltage and calculate SoC (%)
    if (BQ25638_ReadReg16(BQ25638_REG_VBAT_ADC, &raw) == HAL_OK) {
        // Bits 12:1 contain VBAT_ADC
        uint16_t vbat_raw = (raw >> 1) & 0x0FFF; // shift out bit 0 (reserved) and mask 12 bits

        // Convert to mV
        float voltage = vbat_raw * 1.25f; // 1.25 mV per step

        // Map voltage to SoC (%)
        // Assume battery range 3.0V (0%) to 4.2V (100%)
        float soc_f = ((voltage / 1000.0f) - 3.0f) / (4.2f - 3.0f) * 100.0f;
        if (soc_f < 0) soc_f = 0;
        if (soc_f > 100) soc_f = 100;

        status.battery_soc = (uint8_t)soc_f;
    } else {
        status.battery_soc = 0xFF; // Error reading voltage
    }

    return status;
}



uint8_t BQ25638_GetChargeStatus(void) {
    uint8_t status;
    if(BQ25638_ReadReg(BQ25638_REG_CHG_STATUS_0, &status) != HAL_OK) return 0xFF;
    return status & 0x07; // Mask CHG_STAT bits
}

// Read fault register
uint8_t BQ25638_GetFault(void) {
    uint8_t val;
    if(BQ25638_ReadReg(BQ25638_REG_FAULT_STATUS, &val) != HAL_OK) return 0xFF;
    return val;
}

//GPIO Helpers

// Control BATT_CE pin (Charge Enable)
void BQ25638_SetCE(uint8_t value) {
    HAL_GPIO_WritePin(BQ25638_BATT_CE_PORT, BQ25638_BATT_CE_PIN, value);
}

// Read BAT_QON pin (Charge QON status)
uint8_t BQ25638_ReadQON(void) {
    return HAL_GPIO_ReadPin(BQ25638_BAT_QON_PORT, BQ25638_BAT_QON_PIN);
}

// Read STAT pin
uint8_t BQ25638_ReadSTAT(void) {
    return HAL_GPIO_ReadPin(BQ25638_STAT_PORT, BQ25638_STAT_PIN);
}

// Read INT pin
uint8_t BQ25638_ReadINT(void) {
    return HAL_GPIO_ReadPin(BQ25638_INT_PORT, BQ25638_INT_PIN);
}

// Read PGOOD pin (Power Good)
uint8_t BQ25638_ReadPG(void) {
    return HAL_GPIO_ReadPin(BQ25638_PG_PORT, BQ25638_PG_PIN);
}

//Interrupt Handling

// User-defined callback to handle INT events
void BQ25638_INT_Callback(void) {
//    uint8_t status = BQ25638_GetChargeStatus();
//    uint8_t fault  = BQ25638_GetFault();
//
//    // Example: Handle charge termination and faults
//    if(status == 0x03) {
//        HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
//    }
//    if(fault != 0) {
//        // Fault handling: add logging or recovery here
//    }
}

//Initialization

// Initialize BQ25638: set CE high, default charge current/voltage
HAL_StatusTypeDef BQ25638_Init(void) {
    BQ25638_SetCE(0);                // Enable charging circuitry
    //BQ25638_SetChargeCurrent(1024);  // Default 1A
    //BQ25638_SetChargeVoltage(4208);  // Default 4.208V
    //BQ25638_SetTerminationCurrent(128); // Default termination
    BQ25638_SetTsIgnore(1);
    BQ25638_EnableAdc(1);
    return BQ25638_EnableCharging();
}

