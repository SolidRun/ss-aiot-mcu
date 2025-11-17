# STM32U031C8 I2C Sensor Slave Project

## Overview

This project implements an I2C slave interface on the STM32U031C8 MCU to control and read multiple sensors. The MCU acts as a slave device, receiving commands from a master over I2C and performing actions such as turning sensors on/off, reading sensor data, and configuring thresholds.

## Supported sensors and modules:

-LED

-Infrared (IR) sensor

-Accelerometer

-GPS

-Battery charger status

-Real-time clock (RTC)

-Interrupts (MCU, IR, ACC)

## Table of Contents:

1. Command Definitions  
  
    1.1 Command & Response Format
  
    1.2 Examples

2. Sensor Thresholds & Configuration

    2.1 Accelerometer

    2.2 Infrared Sensor (IR)

    2.3 Battery Charging




### 1. Command Definitions:
[Protocol Header](Core/Inc/protocol.h)

### 1.1 Command & Response Format:

Command structure:
| CMD (1B) | SENSOR_ID (1B) | DATA_LEN (1B) | DATA (N Bytes) |

Response structure:
| STATUS (1B) | DATA_LEN (1B) | DATA (N Bytes) |

### 1.2 Examples:

| Command Description           | Command                  | Expected Response                           |
|-------------------------------|--------------------------|--------------------------------------------|
| Turn ON LED                    | {0x10,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Turn OFF LED                   | {0x11,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Read LED status                | {0x12,0x01,0x00,{}}     | {0x00,1,{0x01}} (0x01=ON, 0x00=OFF)        |
| Read IR data                   | {0x12,0x02,0x00,{}}     | {0x00,5,{INT, int16 presenceVal, int16 motionVal}}       |
| Set IR threshold               | {0x13,0x02,0x01,{THS}}  | {0x00,0x00,{}}                             |
| Read acceleration              | {0x12,0x03,0x00,{}}     | {0x00,1,{INT}}                            |
| Set ACC threshold              | {0x13,0x03,0x01,{THS}}  | {0x00,0x00,{}}                             |
| Read GPS data                  | {0x12,0x04,0x00,{}}     | {0x00,12,{lat, lon, hMSL}}                |
| Read battery status            | {0x12,0x05,0x00,{}}     | {0x00,3,{power_source, battery_soc, charge_status}} |
| Read current time              | {0x12,0x06,0x00,{}}     | {0x00,6,{YY,MM,DD,HH,MM,SS}}              |
| RTC with GPS                   | {0x13,0x06,0x00,{}}     | {0x00,0x00,{}}                             |
| Read MCU/IR/ACC interrupts     | {0x12,0x07,0x00,{}}     | {0x00,3,{MCU, IR, ACC}}                    |



### 2. Sensor Thresholds & Configuration:

### 2.1 Accelerometer:

Interrupt code:

| Code | Meaning             |
|------|-------------------|
| 0x00 | No motion detected |
| 0x01 | Motion detected    |


Default threshold: ACC_THS_DEFAULT = 0x04

Trigger values: 0–63 (1 LSB = fraction of ±2g full scale)

Threshold (mg) = FS(g) × (threshold / 64) × 1000

| Threshold | FS(g) = ±2g (mg) |
|-----------|-----------------|
| 1         | 31 mg           |
| 2         | 62 mg           |
| 4         | 125 mg          |
| 8         | 250 mg          |
| 16        | 500 mg          |
| 32        | 1 g             |
| 63        | 1.97 g          |
      

##### Recommended values:

1–3: Very sensitive (tiny motion)

4–8: Medium motion (walking, light shake)

10–20: Strong motion (hit, fall)


### 2.2 Infrared Sensor (IR):

Interrupt code:

| Code | Meaning             |
|------|-------------------|
| 0x00 | No motion detected |
| 0x02 | Motion detected    |
| 0x04 | Presence detected    |

Frequency [Hz]= 1Hz = 1000ms
Hysteresis Default (HYST = 32)

Detection:

Compares two internally filtered signals

Event flag set if difference > threshold

Flag cleared when signal < (threshold − hysteresis)

| Threshold | Hysteresis | Approx. Signal Change | Use Case                           |
|-----------|------------|---------------------|-----------------------------------|
| 100       | 32         | ≥0.05°C             | Very sensitive, long range        |
| 150       | 32         | ≥0.075°C            | Sensitive, moderate range         |
| 200       | 32         | ≥0.1°C              | Balanced sensitivity/stability    |
| 250       | 32         | ≥0.125°C            | Less sensitive, shorter range     |
| 300       | 32         | ≥0.15°C             | Low sensitivity, short range      |
| 400       | 32         | ≥0.2°C              | Minimal sensitivity, very stable  |


### 2.3 Battery Charging:

| Power Source Code | Meaning                 |
|------------------|------------------------|
| 000b             | Not powered from VBUS  |
| 100b             | Unknown adapter        |
| 111b             | Boost OTG              |


| Charge Status Code | Meaning                         |
|-------------------|---------------------------------|
| 000               | Not charging                     |
| 001               | Trickle charge                   |
| 010               | Pre-charge                       |
| 011               | Fast charge (CC mode)            |
| 100               | Taper charge (CV mode)           |
| 110               | Top-off timer active charging    |
| 111               | Charge termination done          |






