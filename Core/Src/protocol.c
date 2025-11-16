/*
 * protocol.c
 *
 *  Created on: Sep 25, 2025
 *      Author: User
 */

#include "protocol.h"
#include "stm32u0xx_hal.h"
#include <string.h>

extern uint8_t ir_ths;
extern uint8_t acc_ths;

void Sensor_LED_On(void) {
	HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_SET);
}

void Sensor_LED_Off(void) {
	HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_RESET);
}

void Sensor_LED_Read(uint8_t *data, uint8_t *len){
	*len = 1;
	if (HAL_GPIO_ReadPin(LED_MCU_GPIO_Port, LED_MCU_Pin)){
		data[0] = 0x01;
	}else{
		data[0] = 0x00;
	}
}

void Sensor_IR_Read(uint8_t *data, uint8_t *len) {
	int16_t presence, motion;
    IR_SENSOR_ReadPresence(&presence);
    IR_SENSOR_ReadMotion(&motion);
    *len = 5;
    data[0] = IR_SENSOR_getInt();
    memcpy(&data[1],  &presence,  sizeof(int16_t));
    memcpy(&data[3],  &motion,  sizeof(int16_t));
    IR_SENSOR_clearInt();
}

void Sensor_IR_Config(uint8_t *cmd_data){
	ir_ths = cmd_data[0]<<8 | cmd_data[1] ;
	IR_SENSOR_InitCtx();
	IR_SENSOR_StartContinuous(STHS34PF80_ODR_AT_1Hz);
}

void Sensor_Accel_Read(uint8_t *data, uint8_t *len) {
    *len = 1;
    data[0] = ACC_getInt();
    ACC_clearInt();
}

void Sensor_Accel_Config(uint8_t *cmd_data){
	acc_ths = cmd_data[0];
	ACC_Init();
}

void Sensor_RTC_Read(uint8_t *data, uint8_t *len) {
	//data:{YY,MM,DD,HH,MM,SS}}

	RTC_TimeTypeDef rtc_Time = {0};
	RTC_DateTypeDef rtc_Date = {0};
	rtc_getTime(&rtc_Time);
	rtc_getDate(&rtc_Date);
	*len= 6;
	data[0] = rtc_Date.Year;
	data[1] = rtc_Date.Month;
	data[2] = rtc_Date.Date;
	data[3] = rtc_Time.Hours;
	data[4] = rtc_Time.Minutes;
	data[5] = rtc_Time.Seconds;

}

void Sensor_RTC_Config(uint8_t *cmd_data){
	SyncRTCWithGPS();
}

void Sensor_Charger_Read(uint8_t *data, uint8_t *len) {
    *len = 3;
    BQ25638_Status_t status;
    status = BQ25638_GetStatus();

    data[0] = status.power_source;
    data[1] = status.battery_soc;
    data[2] = status.charge_status;
}

void Sensor_GPS_Read(uint8_t *data, uint8_t *len) {
    *len = 12;
	UBX_NAV_PVT_t nav;
	UBlox_ReadNavPvt(&nav);
	memcpy(&data[0],  &nav.lat,  sizeof(nav.lat));
	memcpy(&data[4],  &nav.lon,  sizeof(nav.lon));
	memcpy(&data[8],  &nav.hMSL, sizeof(nav.hMSL));
}

void INT_Read(uint8_t *data, uint8_t *len) {
    *len = 3;
    data[0] = somGetInt();
    somClearINT();

    data[1] = IR_SENSOR_getInt();
    IR_SENSOR_clearInt();

    data[2] = ACC_getInt();
    ACC_clearInt();
}

/* Protocol command processor
 */
void Protocol_ProcessCommand(I2C_Command_t *cmd, I2C_Response_t *resp) {
    resp->status = 0;
    resp->data_len = 0;

    switch (cmd->cmd) {
        case CMD_SENSOR_ON:
            if (cmd->sensor_id == SENSOR_LED) {
                Sensor_LED_On();
            }
            break;

        case CMD_SENSOR_OFF:
            if (cmd->sensor_id == SENSOR_LED) {
                Sensor_LED_Off();
            }
            break;

        case CMD_SENSOR_READ:
            switch (cmd->sensor_id) {
                case SENSOR_LED:
                	Sensor_LED_Read(resp->data, &resp->data_len);
                	break;
                case SENSOR_RTC:
                	Sensor_RTC_Read(resp->data, &resp->data_len);
                	break;
                case SENSOR_IR:
                    Sensor_IR_Read(resp->data, &resp->data_len);
                    break;
                case SENSOR_ACCELEROMETER:
                    Sensor_Accel_Read(resp->data, &resp->data_len);
                    break;
                case SENSOR_GPS:
                    Sensor_GPS_Read(resp->data, &resp->data_len);
                    break;
                case SENSOR_BATTERY_CHARGER:
                	Sensor_Charger_Read(resp->data, &resp->data_len);
                    break;
                case INTERRUPTS:
                	INT_Read(resp->data, &resp->data_len);
                    break;
                default:
                    resp->status = 1; // Unknown sensor
                    break;
            }
            break;
        case CMD_SENSOR_CONFIG:
        	switch (cmd->sensor_id) {
        		case SENSOR_IR:
        			Sensor_IR_Config(cmd->data);
        			break;
        		case SENSOR_ACCELEROMETER:
        			Sensor_Accel_Config(cmd->data);
        			break;
        		case SENSOR_RTC:
        			Sensor_RTC_Config(cmd->data);
        			break;
                default:
                    resp->status = 1; // Unknown sensor
                    break;
        	}
        	break;
        default:
            resp->status = 1; // Unknown command
            break;
    }
}

