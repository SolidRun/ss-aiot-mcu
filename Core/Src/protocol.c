/*
 * protocol.c
 *
 *  Created on: Sep 25, 2025
 *      Author: User
 */

#include "protocol.h"
#include "stm32u0xx_hal.h"
#include "nmea.h"
#include <string.h>

extern uint8_t acc_ths;
extern volatile bool gps_time_synced;
extern volatile bool gps_time_sync_request;

void Sensor_LED_On(void) {
	HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_SET);
}

void Sensor_LED_Off(void) {
	HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_RESET);
}

void Sensor_LED_Read(uint8_t *data, uint8_t *len, uint8_t *status){
	*len = 1;
	*status = 0;
	if (HAL_GPIO_ReadPin(LED_MCU_GPIO_Port, LED_MCU_Pin)){
		data[0] = 0x01;
	}else{
		data[0] = 0x00;
	}
}

/* Response is six bytes, every field int16 little-endian:
 *
 *   | Byte | Field    | Encoding                                  |
 *   |------|----------|-------------------------------------------|
 *   | 0-1  | presence | raw algorithm output                      |
 *   | 2-3  | motion   | raw algorithm output                      |
 *   | 4-5  | tamb     | ambient temperature, hundredths of degC   |
 *
 * tamb is the STHS34PF80's own ambient channel at 100 LSB/degC, so the raw
 * value already is hundredths - it is passed straight through. Independent
 * of the accelerometer's temperature, which makes the two a cross-check.
 *
 * Note: this runs in the I2C2 slave callback and reads I2C1 directly,
 * which the accelerometer path deliberately no longer does. Three bus
 * transactions here now instead of two. Same class of problem as before,
 * one transaction worse; the IR path wants the same cache treatment.
 */
void Sensor_IR_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
	int16_t presence, motion, tamb;
    IR_SENSOR_ReadPresence(&presence);
    IR_SENSOR_ReadMotion(&motion);
    IR_SENSOR_ReadTAmbient(&tamb);
    *status = 0;
    *len = 6;
    memcpy(&data[0],  &presence,  sizeof(int16_t));
    memcpy(&data[2],  &motion,  sizeof(int16_t));
    memcpy(&data[4],  &tamb,  sizeof(int16_t));
}

void Sensor_IR_Config(uint8_t *cmd_data){
	ir_ths = (uint16_t)(((uint16_t)cmd_data[0] << 8) | cmd_data[1]);
	IR_SENSOR_InitCtx();
	IR_SENSOR_StartContinuous(STHS34PF80_ODR_AT_1Hz);
}

/* Response is nine bytes:
 *
 *   | Byte | Field  | Encoding                                      |
 *   |------|--------|-----------------------------------------------|
 *   | 0    | reason | event bits, as before this command grew       |
 *   | 1-2  | x      | int16, mg, little-endian                      |
 *   | 3-4  | y      | int16, mg, little-endian                      |
 *   | 5-6  | z      | int16, mg, little-endian                      |
 *   | 7-8  | temp   | int16, hundredths of degC, little-endian      |
 *
 * reason bits: 0x01 Z, 0x02 Y, 0x04 X, 0x08 wake-up, 0x10 tilt,
 * 0x20 free-fall, 0x40 sleep change.
 *
 * Every read also asks the sensor EXTI handler for a new sample, so the
 * axes are the ones the previous read asked for, not this one's.
 *
 * STATUS says how much to trust them, and never withholds them:
 *
 *   0  the sample is under a second old
 *   1  nothing has ever been sampled; the axes are zero and mean nothing
 *   2  the sample is real but over a second old
 *
 * 2 is the normal answer for a master polling slower than a second - the
 * sample is one poll interval old by construction. It is still the best
 * value available, and matters because the cache can fall behind without
 * any interrupt firing: the wake-up detector sees change, not position, so
 * a slow tilt moves the device without waking anything.
 *
 * The reason byte is valid under all three.
 */
void Sensor_Accel_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    int16_t xyz[3] = {0, 0, 0};
    int16_t temp   = 0;

    *len = 9;
    data[0] = (uint8_t)ACC_getInt();

    /* the return value is the status byte; the outputs stay zero when 1 */
    *status = (uint8_t)ACC_GetCachedAxes(xyz, &temp);

    data[1] = (uint8_t)((uint16_t)xyz[0] & 0xff);
    data[2] = (uint8_t)((uint16_t)xyz[0] >> 8);
    data[3] = (uint8_t)((uint16_t)xyz[1] & 0xff);
    data[4] = (uint8_t)((uint16_t)xyz[1] >> 8);
    data[5] = (uint8_t)((uint16_t)xyz[2] & 0xff);
    data[6] = (uint8_t)((uint16_t)xyz[2] >> 8);
    data[7] = (uint8_t)((uint16_t)temp   & 0xff);
    data[8] = (uint8_t)((uint16_t)temp   >> 8);
}

/* Re-runs the whole accelerometer bring-up with a new wake-up threshold.
 * STATUS is 0 on success, otherwise the number of the ACC_Init() step that
 * failed - so this command doubles as a way to ask whether the sensor is
 * actually configured, which nothing could do before. */
void Sensor_Accel_Config(uint8_t *cmd_data, uint8_t *status){
	acc_ths = cmd_data[0];
	*status = (uint8_t)ACC_Init();
}

void Sensor_RTC_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
	//data:{YY,MM,DD,HH,MM,SS}}
	if (!gps_time_synced){
		*status= 1;
	}else{
		*status = 0;
	}

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

void Sensor_RTC_Config(uint8_t *cmd_data ,uint8_t *status){
	*status = 0;
	gps_time_sync_request = true ;
}

/* 0x13 0x08 - arm the daily alarm from {HH, MM, SS}, binary, 24-hour.
 *
 * Cancelling is 0x11 0x08, not a magic time value, so 00:00:00 is settable like
 * any other time. */
void Sensor_Alarm_Config(uint8_t *cmd_data, uint8_t data_len, uint8_t *status)
{
	if (data_len != 3U) {
		*status = 1;
		return;
	}

	if ((cmd_data[0] > 23U) || (cmd_data[1] > 59U) || (cmd_data[2] > 59U)) {
		*status = 1;
		return;
	}

	*status = rtc_setDailyAlarm(cmd_data[0], cmd_data[1], cmd_data[2]) ? 0U : 1U;
}

/* 0x12 0x08 - report the armed alarm.
 *
 * STATUS is 0 when an alarm is set and armed.
 * STATUS is 1 when no alarm is active or was never set.
 */
void Sensor_Alarm_Read(uint8_t *data, uint8_t *len, uint8_t *status)
{
	*len = 3;

    if (rtc_getAlarm(&data[0], &data[1], &data[2]))
        *status = 0;
    else
        *status = 1;
}

void Sensor_Charger_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    *len = 7;
    BQ25638_Status_t BQ_status;

    if (BQ25638_GetStatus(&BQ_status) == HAL_OK) {
        /* The three measurements are 16-bit, little-endian, low byte first.
         * ibat is signed - two's complement, as the charger reports it. */
        data[0] = BQ_status.flags;
        data[1] = (uint8_t)((uint16_t)BQ_status.ibat & 0xff);
        data[2] = (uint8_t)((uint16_t)BQ_status.ibat >> 8);
        data[3] = (uint8_t)(BQ_status.vbat & 0xff);
        data[4] = (uint8_t)(BQ_status.vbat >> 8);
        data[5] = (uint8_t)(BQ_status.vbus & 0xff);
        data[6] = (uint8_t)(BQ_status.vbus >> 8);
        *status = 0;
    } else {
        /* On error return non-zero status, data is now invalid and master must discard it. */
        *status = 1;
    }
}

void Sensor_GPS_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    uint8_t n = 0;

    /* Fill the whole payload from as many queued sentences as fit. The response
     * length has to be constant. The master cannot learn DATA_LEN before it
     * reads, and reading past the armed length leaves the slave stretching SCL
     * with nothing left to send - the bus hangs until the stuck-bus watchdog
     * fires seconds later. So the length is fixed and the master always reads
     * 2 + GPS_CHUNK_MAX. */
    while (n < GPS_CHUNK_MAX) {
        uint8_t got = NMEA_Pop(&data[n], (uint8_t)(GPS_CHUNK_MAX - n));
        if (got == 0U) {
            break;              /* queue drained */
        }
        n += got;
    }

    /* An empty queue is reported in STATUS, not in the length. */
    *status = (n > 0U) ? 0U : 1U;

    /* Pad with newlines - empty lines, which any NMEA framer discards. Padding
     * can only ever follow a complete sentence: NMEA_Pop returns a partial one
     * only when it filled the payload, and then there is nothing left to pad. */
    while (n < GPS_CHUNK_MAX) {
        data[n++] = (uint8_t)'\n';
    }

    *len = GPS_CHUNK_MAX;
}

void Sensor_GPS_Config(uint8_t *cmd_data){

	int RSTN_PinState = cmd_data[0];
	int EN_PinState = cmd_data[1];
	HAL_GPIO_WritePin(GPIOB, GPS_RSTN_Pin, RSTN_PinState);
	HAL_GPIO_WritePin(GNSS_PWR_EN_GPIO_Port, GNSS_PWR_EN_Pin, EN_PinState);
}

void INT_Read(uint8_t *data, uint8_t *len) {
    *len = 4;
    somTakeInterrupts(&data[0], &data[1], &data[2], &data[3]);
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
            } else if (cmd->sensor_id == SENSOR_ALARM) {
                rtc_cancelAlarm();
            } else if (cmd->sensor_id == SENSOR_SOM) {
                /* schedule power-off after 1s */
                SomScheduleOff(1000);
                /* status is success (master can't process failure during shutdown anyhow) */
            }
            break;

        case CMD_SENSOR_READ:
            switch (cmd->sensor_id) {
                case SENSOR_LED:
                	Sensor_LED_Read(resp->data, &resp->data_len, &resp->status);
                	break;
                case SENSOR_RTC:
                	Sensor_RTC_Read(resp->data, &resp->data_len, &resp->status);
                	break;
                case SENSOR_IR:
                    Sensor_IR_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_ACCELEROMETER:
                    Sensor_Accel_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_GPS:
                    Sensor_GPS_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_BATTERY_CHARGER:
                	Sensor_Charger_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_ALARM:
                    Sensor_Alarm_Read(resp->data, &resp->data_len, &resp->status);
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
        			Sensor_Accel_Config(cmd->data, &resp->status);
        			break;
        		case SENSOR_RTC:
        			Sensor_RTC_Config(cmd->data , &resp->status);
        			break;
        		case SENSOR_GPS:
        			Sensor_GPS_Config(cmd->data);
        			break;
        		case SENSOR_ALARM:
        			Sensor_Alarm_Config(cmd->data, cmd->data_len, &resp->status);
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

