// SPDX-License-Identifier: BSD-3-Clause
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

/* Regenerated before every build by tools/build_id.sh, the pre-build step in
 * .cproject. Included here alone, so a new commit rebuilds this object only. */
#include "build_id.h"

extern volatile bool gps_time_synced;
extern volatile bool gps_time_sync_request;

/* As Sensor_Accel_Motion_Read: a snapshot of the sample timebase, then up to
 * IR_SAMPLES_PER_READ buffered samples. */
void Sensor_IR_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    /* snapshot the sample timebase */
    uint32_t timebase = IR_TimestampNow();

    /* little-endian, and the tx buffer is not word aligned */
    memcpy(&data[0], &timebase, SAMPLE_TIMEBASE_LEN);

    /* get buffered IR samples, max. IR_SAMPLES_PER_READ */
    size_t n = IR_TakeSamples((ir_sample_t *)&data[SAMPLE_TIMEBASE_LEN],
                              IR_SAMPLES_PER_READ);

    /* calculate length in bytes, snapshot included */
    *len = (uint8_t)(SAMPLE_TIMEBASE_LEN + n * sizeof(ir_sample_t));

    /* this command can't fail and no samples is not an error, set status 0 */
    *status = 0;
}

/* 0x13 0x02 - report the IR sensor configuration, ir_config_t in wire order. */
void Sensor_IR_Config(uint8_t *cmd_data, uint8_t cmd_len, uint8_t *data,
                      uint8_t *len, uint8_t *status) {
    /* ignore inputs, set not implemented */
    (void)cmd_data;
    (void)cmd_len;

    /* struct is packed, read config directly into tx buffer */
    IR_GetConfig((ir_config_t *)data);

    *len = (uint8_t)IR_CONFIG_LEN;
    *status = 0;
}

void Sensor_Accel_Motion_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    /* snapshot the sample timebase */
    uint32_t timebase = ACC_TimestampNow();

    /* little-endian, and the tx buffer is not word aligned */
    memcpy(&data[0], &timebase, SAMPLE_TIMEBASE_LEN);

    /* get raw accelerometer samples, max. ACC_MOTION_SAMPLES_PER_READ */
    size_t n = ACC_TakeMotionSamples((acc_motionsample_t *)&data[SAMPLE_TIMEBASE_LEN],
                                     ACC_MOTION_SAMPLES_PER_READ);

    /* calculate length in bytes, snapshot included */
    *len = (uint8_t)(SAMPLE_TIMEBASE_LEN + n * sizeof(acc_motionsample_t));

    /* this command can't fail and no samples is not an error, set status 0 */
    *status = 0;
}

void Sensor_Accel_Temp_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    /* snapshot the sample timebase */
    uint32_t timebase = ACC_TimestampNow();

    /* little-endian, and the tx buffer is not word aligned */
    memcpy(&data[0], &timebase, SAMPLE_TIMEBASE_LEN);

    /* get one raw die temperature sample */
    size_t n = ACC_TakeTempSamples((acc_tempsample_t *)&data[SAMPLE_TIMEBASE_LEN],
                                   ACC_TEMP_SAMPLES_PER_READ);

    /* calculate length in bytes, snapshot included */
    *len = (uint8_t)(SAMPLE_TIMEBASE_LEN + n * sizeof(acc_tempsample_t));

    /* this command can't fail and no sample is not an error, set status 0 */
    *status = 0;
}

/* 0x13 0x03 - placeholder. Accepts and discards its payload; the wake-up
 * threshold is fixed at ACC_THS_DEFAULT. */
void Sensor_Accel_Config(uint8_t *cmd_data, uint8_t cmd_len, uint8_t *status){
	(void)cmd_data;
	(void)cmd_len;
	*status = 0;
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

/* 0x13 0x08 - read or replace the alarm configuration, {FLAGS, HH, MM, SS}.
 *
 * An empty payload reads it; a full one replaces it first. The response
 * carries the effective configuration.
 */
void Sensor_Alarm_Config(uint8_t *cmd_data, uint8_t cmd_len, uint8_t *data,
                         uint8_t *len, uint8_t *status)
{
    static const uint8_t alrm_cfg_len = 4;

    *status = 0;

    if (cmd_len == 0) {
        /* read, no-op */
    } else if (cmd_len == alrm_cfg_len) {
		if (cmd_data[0] & ALARM_FLAG_ARMED) {
	        if (!rtc_setDailyAlarm(cmd_data[1], cmd_data[2], cmd_data[3]))
		        *status = 1;
        } else {
            rtc_cancelAlarm();
        }
	} else {
        /* invalid payload length */
        *status = 1;
    }

    /* always return effective status */
	*len = alrm_cfg_len;

    /* init flags */
    data[0] = 0;

    /* get alarm status and time */
    if (rtc_getAlarm(&data[1], &data[2], &data[3])) {
        /* alarm active */
        data[0] |= ALARM_FLAG_ARMED;
    }
}

/* Pass through the reading BQ25638_Process() cached, no bus access. */
void Sensor_Charger_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    *len = 7;
    BQ25638_Status_t BQ_status;

    if (BQ25638_GetLastStatus(&BQ_status)) {
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
        /* The last read failed or none has been taken yet, data is invalid
         * and master must discard it. */
        *status = 1;
    }
}

void Sensor_GPS_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    uint8_t n = 0;

    /* Fill the whole payload from as many queued sentences as fit. */
    while (n < GPS_CHUNK_MAX) {
        uint8_t got = NMEA_Pop(&data[n], (uint8_t)(GPS_CHUNK_MAX - n));
        if (got == 0U) {
            break;              /* queue drained */
        }
        n += got;
    }

    /* An empty queue is reported in STATUS. */
    *status = (n > 0U) ? 0U : 1U;
    /* length indicates real data size, however data is still padded t max length */
    *len = n;

    /*
     * The protocol demands padding up to GPS_CHUNK_MAX, which allows a master
     * to use fixed-size reads and ignore data_len field.
     * Pad with newlines - empty lines, which any NMEA framer discards. Padding
     * can only ever follow a complete sentence: NMEA_Pop returns a partial one
     * only when it filled the payload, and then there is nothing left to pad. */
    while (n < GPS_CHUNK_MAX) {
        data[n++] = (uint8_t)'\n';
    }
}

/* 0x13 0x04 - placeholder. Accepts and discards its payload; GPS_RSTN and
 * GNSS_PWR_EN stay as MX_GPIO_Init() left them, powered and out of reset. */
void Sensor_GPS_Config(uint8_t *cmd_data, uint8_t cmd_len, uint8_t *status){
	(void)cmd_data;
	(void)cmd_len;
	*status = 0;
}

/* 0x12 0x0B - report what this firmware is: the protocol version it speaks and
 * the commit it was built from. */
void MCU_Info_Read(uint8_t *data, uint8_t *len, uint8_t *status) {
    uint32_t build_id = BUILD_ID;

    data[0] = MCU_API_VERSION;
    data[1] = 0U;

    /* the generator defines this only when the tree was dirty */
#ifdef BUILD_ID_DIRTY
    data[1] |= MCU_FLAG_BUILD_DIRTY;
#endif

    /* little-endian, and the tx buffer is not word aligned */
    memcpy(&data[2], &build_id, sizeof(build_id));

    *len = MCU_INFO_LEN;
    *status = 0;
}

void INT_Read(uint8_t *data, uint8_t *len) {
    *len = 5;
    somTakeInterrupts(&data[0], &data[1], &data[2], &data[3], &data[4]);
}

/* 0x13 0x07 - read or replace the interrupt configuration.
 *
 * An empty payload reads it; a full one replaces it first. Either way the
 * response carries the configuration now in effect, so a master sees what took
 * hold rather than what it asked for.
 *
 * Bits set in PWR_SOURCES for a source that EN_SOURCES switches off have no
 * effect; the source is not reported at all.
 */
void INT_Config(uint8_t *cmd_data, uint8_t cmd_len, uint8_t *data, uint8_t *len,
                uint8_t *status) {
    if ((cmd_len != 0U) && (cmd_len != INT_CONFIG_LEN)) {
        /* neither a read nor a whole configuration, change nothing */
        *status = 1;
        *len = 0;
        return;
    }

    if (cmd_len == INT_CONFIG_LEN) {
        somSetIntConfig(cmd_data);
    }

    somGetIntConfig(data);
    *len = INT_CONFIG_LEN;
    *status = 0;
}

/* Protocol command processor
 */
void Protocol_ProcessCommand(I2C_Command_t *cmd, I2C_Response_t *resp) {
    resp->status = 0;
    resp->data_len = 0;

    switch (cmd->cmd) {
        case CMD_SENSOR_OFF:
        	if (cmd->sensor_id == SENSOR_SOM) {
                /* schedule power-off after 1s */
                SomScheduleOff(1000);
                /* status is success (master can't process failure during shutdown anyhow) */
            }
            break;

        case CMD_SENSOR_READ:
            switch (cmd->sensor_id) {
                case SENSOR_RTC:
                	Sensor_RTC_Read(resp->data, &resp->data_len, &resp->status);
                	break;
                case SENSOR_IR:
                    Sensor_IR_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_ACCEL_MOTION:
                    Sensor_Accel_Motion_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_ACCEL_TEMP:
                    Sensor_Accel_Temp_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_GPS:
                    Sensor_GPS_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case SENSOR_BATTERY_CHARGER:
                	Sensor_Charger_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                case INTERRUPTS:
                	INT_Read(resp->data, &resp->data_len);
                    break;
                case SENSOR_MCU:
                    MCU_Info_Read(resp->data, &resp->data_len, &resp->status);
                    break;
                default:
                    resp->status = 1; // Unknown sensor
                    break;
            }
            break;
        case CMD_SENSOR_CONFIG:
        	switch (cmd->sensor_id) {
        		case SENSOR_IR:
        			Sensor_IR_Config(cmd->data, cmd->data_len, resp->data, &resp->data_len, &resp->status);
        			break;
        		case SENSOR_ACCEL_MOTION:
        			Sensor_Accel_Config(cmd->data, cmd->data_len, &resp->status);
        			break;
        		case SENSOR_RTC:
        			Sensor_RTC_Config(cmd->data , &resp->status);
        			break;
        		case SENSOR_GPS:
        			Sensor_GPS_Config(cmd->data, cmd->data_len, &resp->status);
        			break;
        		case SENSOR_ALARM:
        			Sensor_Alarm_Config(cmd->data, cmd->data_len, resp->data,
        			                    &resp->data_len, &resp->status);
        			break;
        		case INTERRUPTS:
        			INT_Config(cmd->data, cmd->data_len, resp->data,
        			           &resp->data_len, &resp->status);
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

