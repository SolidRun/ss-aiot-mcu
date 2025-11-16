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
    SENSOR_LED              = 0x01, // LED control
    SENSOR_IR               = 0x02, // Infrared sensor
    SENSOR_ACCELEROMETER   = 0x03, // Accelerometer sensor
	SENSOR_GPS			   = 0x04,  //GPA sensor
    SENSOR_BATTERY_CHARGER = 0x05, // Battery charger status
    SENSOR_RTC              = 0x06,  // Real-time clock
	INTERRUPTS              =0x07
} SensorID_t;

/* Command structure
 * Defines the format of a command sent over I2C.
 */
typedef struct {
    uint8_t cmd;        // Command code
    uint8_t sensor_id;  // Target sensor/module ID
    uint8_t data_len;   // Length of data payload
    uint8_t data[32];   // Data payload (max 32 bytes)
} I2C_Command_t;

/* Response structure
 * Defines the format of a response from the I2C slave.
 */
typedef struct {
    uint8_t status;     // 0 = OK, 1 = ERROR
    uint8_t data_len;   // Length of response data
    uint8_t data[32];   // Response payload
} I2C_Response_t;

/* API
 * Processes a received command and prepares a response.
 */
void Protocol_ProcessCommand(I2C_Command_t *cmd, I2C_Response_t *resp);

#endif // PROTOCOL_H

