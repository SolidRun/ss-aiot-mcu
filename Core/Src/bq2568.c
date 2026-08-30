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

/*
 * Populate passed BQ25638_Status_t structure with status information.
 * Returns HAL_OK on success, HAL_ERROR on error.
 *
 * On error, status values are partially uninitialised, caller should discard.
 */
HAL_StatusTypeDef BQ25638_GetStatus(BQ25638_Status_t *out) {

    uint8_t  chg_status_0, chg_status_1, fault_status;
    uint16_t adc_reg_ibat, adc_reg_vbat, adc_reg_vbus;
    int32_t  adc_val_ibat;
    uint32_t adc_val_vbat, adc_val_vbus;

    /* zero flags */
    out->flags = 0;

    /* read charger status register 0 */
    if (BQ25638_ReadReg(BQ25638_REG_CHG_STATUS_0, &chg_status_0) != HAL_OK) {
        return HAL_ERROR;
    }

    /* check if mains present (PG_STAT (bit 7) vbus power-good) */
    if (chg_status_0 & BQ25638_REG_CHG_STATUS_0__PG_STAT)
        out->flags |= BQ25638_FLAG_MAINS;

    /* read charger status register 1 */
    if (BQ25638_ReadReg(BQ25638_REG_CHG_STATUS_1, &chg_status_1) != HAL_OK) {
        return HAL_ERROR;
    }

    /* check if charging (two values from CHG_STAT mean not charging, all others mean charging) */
    if (((chg_status_1 & BQ25638_REG_CHG_STATUS_1__CHG_STAT_MASK) != BQ25638_REG_CHG_STATUS_1__CHG_STAT__NOT_CHARGING) &&
        ((chg_status_1 & BQ25638_REG_CHG_STATUS_1__CHG_STAT_MASK) != BQ25638_REG_CHG_STATUS_1__CHG_STAT__TERMINATION_DONE))
        out->flags |= BQ25638_FLAG_CHARGING;

    /* read charger fault status register */
    if (BQ25638_ReadReg(BQ25638_REG_FAULT_STATUS, &fault_status) != HAL_OK) {
        return HAL_ERROR;
    }

    /* check if mains has fault (over-voltage) */
    if (fault_status & BQ25638_REG_FAULT_STATUS__VBUS_FAULT_STAT)
        out->flags |= BQ25638_FLAG_MAINS_FAULT;

    /* check if battery has fault (dead or over-voltage) */
    if (fault_status & BQ25638_REG_FAULT_STATUS__VBUS_FAULT_STAT)
        out->flags |= BQ25638_FLAG_BAT_FAULT;

    /* read adc ibat register */
    if (BQ25638_ReadReg16(BQ25638_REG_IBAT_ADC, &adc_reg_ibat) != HAL_OK) {
        return HAL_ERROR;
    }

    /* unpack ibat adc value (bits 3-15, 2s complement) and scale to mA (raw 5mA per bit) */
    adc_val_ibat = (int16_t)adc_reg_ibat >> 3;
    adc_val_ibat = adc_val_ibat * 5;

    /* return ibat adc value in mA (valid range -10000 - +5025) */
    out->ibat = adc_val_ibat;

    /* read adc vbat register */
    if (BQ25638_ReadReg16(BQ25638_REG_VBAT_ADC, &adc_reg_vbat) != HAL_OK) {
        return HAL_ERROR;
    }

    /* unpack vbat adc value (bits 1-12) and scale to mV (raw 1.25mV per bit) */
    adc_val_vbat = (adc_reg_vbat & 0x1ffe) >> 1;
    adc_val_vbat = (adc_val_vbat * 5) / 4;

    /* return vbat adc value in mV (valid range 0 - 5000) */
    out->vbat = adc_val_vbat;

    /* read adc vbus register */
    if (BQ25638_ReadReg16(BQ25638_REG_VBUS_ADC, &adc_reg_vbus) != HAL_OK) {
        return HAL_ERROR;
    }

    /* unpack vbus adc value (bits 2-14) and scale to mV (raw 5mV per bit) */
    adc_val_vbus = (adc_reg_vbus & 0x7ffc) >> 2;
    adc_val_vbus = adc_val_vbus * 5;

    /* return vbus adc value in mV (valid range 0 - 20000) */
    out->vbus = adc_val_vbus;

    return HAL_OK;
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
    HAL_StatusTypeDef status;

    BQ25638_SetCE(0);                // Enable charging circuitry
    //BQ25638_SetChargeCurrent(1024);  // Default 1A
    //BQ25638_SetChargeVoltage(4208);  // Default 4.208V
    //BQ25638_SetTerminationCurrent(128); // Default termination
    status = BQ25638_SetTsIgnore(1);
    if (status != HAL_OK)
        return status;

    BQ25638_EnableAdc(1);
    if (status != HAL_OK)
        return status;

    return BQ25638_EnableCharging();
}

