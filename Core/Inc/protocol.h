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

/* maximum length of i2c command (write) including command and data */
#define I2C_CMD_MAX_LEN 35

/* minimum length of i2c command (write) excluding data */
#define I2C_CMD_MIN_LEN sizeof(I2C_Command_t)

/* calculate maximum command payload size */
#define I2C_CMD_MAX_PAYLOAD (I2C_CMD_MAX_LEN - I2C_CMD_MIN_LEN)

/* Response structure
 * Defines the format of a response from the I2C slave.
 */
typedef struct {
    uint8_t status;     // 0 = OK, 1 = ERROR
    uint8_t data_len;   // Length of response data
    uint8_t data[];     // Response data
} I2C_Response_t;

/* Maximum length of i2c response (read) including status and data.*/
#define I2C_RESP_MAX_LEN  36

/* minimum length of i2c response (read) excluding data */
#define I2C_RESP_MIN_LEN sizeof(I2C_Response_t)

/* calculate maximum response payload size */
#define I2C_RESP_MAX_PAYLOAD (I2C_RESP_MAX_LEN - I2C_RESP_MIN_LEN)

/* Max GPS bytes per read - the response payload size. A full NMEA sentence is
 * longer than this, so a sentence may span several reads. */
#define GPS_CHUNK_MAX 32U

/* ensure gps chunk size agrees with max payload size */
_Static_assert(GPS_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "GPS_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* size of the timebase snapshot every sample read opens with */
#define SAMPLE_TIMEBASE_LEN sizeof(uint32_t)

/* maximum accelerometer motion data samples per read (payload size) */
#define ACC_MOTION_SAMPLES_PER_READ 3
#define ACC_MOTION_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + ACC_MOTION_SAMPLES_PER_READ * sizeof(acc_motionsample_t))

/* ensure accelerometer motion data read size agrees with max payload size */
_Static_assert(ACC_MOTION_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "ACC_MOTION_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* maximum accelerometer temperature samples per read (payload size) */
#define ACC_TEMP_SAMPLES_PER_READ 1
#define ACC_TEMP_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + ACC_TEMP_SAMPLES_PER_READ * sizeof(acc_tempsample_t))

/* ensure accelerometer temperature data read size agrees with max payload size */
_Static_assert(ACC_TEMP_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "ACC_TEMP_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* maximum IR samples per read (payload size) */
#define IR_SAMPLES_PER_READ 3
#define IR_CHUNK_MAX (SAMPLE_TIMEBASE_LEN + IR_SAMPLES_PER_READ * sizeof(ir_sample_t))

/* ensure IR data read size agrees with max payload size */
_Static_assert(IR_CHUNK_MAX <= I2C_RESP_MAX_PAYLOAD, "IR_CHUNK_MAX > I2C_RESP_MAX_PAYLOAD");

/* API
 * Processes a received command and prepares a response.
 */

void Protocol_ProcessCommand(I2C_Command_t *cmd, I2C_Response_t *resp);

#endif // PROTOCOL_H

