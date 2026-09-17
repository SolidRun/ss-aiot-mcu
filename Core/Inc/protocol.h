// SPDX-License-Identifier: BSD-3-Clause
/*
 * protocol.h
 *
 *  Created on: Sep 25, 2025
 *      Author: User
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include "stdio.h"
#include "ir_sensor.h"
#include "acc_sensor.h"
#include "ublox.h"
#include "bq25638.h"
#include "rtc.h"
/* Command definitions
 * These commands are used to control sensors over I2C.
 */
typedef enum {
    CMD_SENSOR_ON      = 0x10, // Turn sensor ON
    CMD_SENSOR_OFF     = 0x11, // Turn sensor OFF
    CMD_SENSOR_READ    = 0x12, // Read data from sensor
    CMD_SENSOR_CONFIG  = 0x13  // Configure sensor settings
} Command_t;

/* Sensor IDs
 * Identifiers for different sensors and modules in the system.
 */
typedef enum {
    SENSOR_LED             = 0x01, // LED control
    SENSOR_IR              = 0x02, // Infrared sensor
    SENSOR_ACCEL_MOTION    = 0x03, // Accelerometer, motion samples
    SENSOR_GPS             = 0x04, // GPS sensor
    SENSOR_BATTERY_CHARGER = 0x05, // Battery charger status
    SENSOR_RTC             = 0x06, // Real-time clock
    INTERRUPTS             = 0x07,
    SENSOR_ALARM           = 0x08, // RTC alarm, a time of day
    SENSOR_SOM             = 0x09, // SOM power rail, Turn OFF only
    SENSOR_ACCEL_TEMP      = 0x0A, // Accelerometer die temperature
    SENSOR_MCU             = 0x0B, // The firmware itself
} SensorID_t;

/* Command structure
 * Defines the format of a command sent over I2C.
 */
typedef struct {
    uint8_t cmd;        // Command code
    uint8_t sensor_id;  // Target sensor/module ID
    uint8_t data_len;   // Length of data payload
    uint8_t data[];     // Command data
} I2C_Command_t;

/* minimum length of i2c command (write) excluding data */
#define I2C_CMD_MIN_LEN sizeof(I2C_Command_t)

/* set maximum command payload size */
#define I2C_CMD_MAX_PAYLOAD UINT8_MAX

/* the payload length travels in data_len, so it has to fit that field */
_Static_assert(I2C_CMD_MAX_PAYLOAD
               <= (1ULL << (8U * sizeof(((I2C_Command_t *)0)->data_len))) - 1ULL,
               "I2C_CMD_MAX_PAYLOAD does not fit I2C_Command_t.data_len");

/* calculate maximum length of i2c command (write) including command and data */
#define I2C_CMD_MAX_LEN (I2C_CMD_MIN_LEN + I2C_CMD_MAX_PAYLOAD)

/* Response structure
 * Defines the format of a response from the I2C slave.
 */
typedef struct {
    uint8_t status;     // 0 = OK, 1 = ERROR
    uint8_t data_len;   // Length of response data
    uint8_t data[];     // Response data
} I2C_Response_t;

/* minimum length of i2c response (read) excluding data */
#define I2C_RESP_MIN_LEN sizeof(I2C_Response_t)

/* set maximum response payload size */
#define I2C_RESP_MAX_PAYLOAD UINT8_MAX

/* the payload length travels in data_len, so it has to fit that field */
_Static_assert(I2C_RESP_MAX_PAYLOAD
               <= (1ULL << (8U * sizeof(((I2C_Response_t *)0)->data_len))) - 1ULL,
               "I2C_RESP_MAX_PAYLOAD does not fit I2C_Response_t.data_len");

/* calculate maximum length of i2c response (read) including status and data */
#define I2C_RESP_MAX_LEN (I2C_RESP_MIN_LEN + I2C_RESP_MAX_PAYLOAD)

/* Max GPS bytes per read - the response payload size. A full NMEA sentence is
 * longer than this, so a sentence may span several reads. */
#define GPS_CHUNK_MAX 32U

/* ensure gps chunk size agrees with max payload size */
_Static_assert(GPS_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "GPS_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* mcu information payload, {API_VERSION, FLAGS, uint32 BUILD_ID} */
#define MCU_INFO_LEN 6U

/* Version of this protocol the firmware implements. Byte 0 of the mcu
 * information payload and the one field whose offset never moves, so a master
 * can read it before it trusts the layout of anything else. */
#define MCU_API_VERSION 0U

/* mcu information flags, byte 1 of the payload */
#define MCU_FLAG_BUILD_DIRTY 0x01U

/* ensure the mcu information fits a response */
_Static_assert(MCU_INFO_LEN <= I2C_RESP_MAX_PAYLOAD, "MCU_INFO_LEN > I2C_RESP_MAX_PAYLOAD");

/* alarm configuration flags, byte 0 of the payload. */
#define ALARM_FLAG_ARMED 0x01U

/* IR configuration response, ir_config_t in wire order */
#define IR_CONFIG_LEN sizeof(ir_config_t)

/* ensure ir config data read size agrees with max payload size */
_Static_assert(IR_CONFIG_LEN <= I2C_RESP_MAX_PAYLOAD, "IR_CONFIG_LEN > I2C_RESP_MAX_PAYLOAD");

/* interrupt configuration payload,
 * {EN_SOURCES, PWR_SOURCES, EN_MCU, EN_IR, EN_ACC, EN_RTC} */
#define INT_CONFIG_LEN 6U

/* ensure the interrupt configuration fits a response and a command */
_Static_assert(INT_CONFIG_LEN <= I2C_RESP_MAX_PAYLOAD, "INT_CONFIG_LEN > I2C_RESP_MAX_PAYLOAD");
_Static_assert(INT_CONFIG_LEN <= I2C_CMD_MAX_PAYLOAD, "INT_CONFIG_LEN > I2C_CMD_MAX_PAYLOAD");

/* size of the timebase snapshot every sample read opens with */
#define SAMPLE_TIMEBASE_LEN sizeof(uint32_t)

/* maximum accelerometer motion data samples per read (payload size) */
#define ACC_MOTION_SAMPLES_PER_READ 8
#define ACC_MOTION_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + ACC_MOTION_SAMPLES_PER_READ * sizeof(acc_motionsample_t))

/* ensure accelerometer motion data read size agrees with max payload size */
_Static_assert(ACC_MOTION_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "ACC_MOTION_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* maximum accelerometer temperature samples per read (payload size) */
#define ACC_TEMP_SAMPLES_PER_READ 1
#define ACC_TEMP_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + ACC_TEMP_SAMPLES_PER_READ * sizeof(acc_tempsample_t))

/* ensure accelerometer temperature data read size agrees with max payload size */
_Static_assert(ACC_TEMP_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "ACC_TEMP_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* maximum IR samples per read (payload size) */
#define IR_SAMPLES_PER_READ 5
#define IR_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + IR_SAMPLES_PER_READ * sizeof(ir_sample_t))

/* ensure IR data read size agrees with max payload size */
_Static_assert(IR_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "IR_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* API
 * Processes a received command and prepares a response.
 */

void Protocol_ProcessCommand(I2C_Command_t *cmd, I2C_Response_t *resp);

#endif // PROTOCOL_H

