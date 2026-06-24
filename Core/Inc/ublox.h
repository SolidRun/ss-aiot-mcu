#ifndef UBLOX_H
#define UBLOX_H

#include "stm32u0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// I2C handle
extern I2C_HandleTypeDef hi2c1;

// UBX message class & ID for NAV-PVT
#define UBX_CLASS_NAV 0x01
#define UBX_ID_PVT   0x07

// UBX header size
#define UBX_HEADER_SIZE 6
#define UBX_CHECKSUM_SIZE 2

#define UBX_TIMEUTC_VALID_MASK  0x04

// UBX NAV-PVT structure (simplified, all values in little-endian)
typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;

    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;

    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;
    int32_t  headMot;

    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flag3;
    uint32_t reserved0;
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;

} UBX_NAV_PVT_t;

typedef struct {
    uint32_t iTOW;    // ms
    int32_t fTOW;     // ns fraction
    int16_t week;     // GPS week
    uint8_t leapS;    // leap seconds
    uint8_t valid;    // validity flags
    uint32_t tAcc;    // accuracy
} UBX_NAV_TIMEGPS_t;
// Functions
void UBlox_Init(void);
bool UBlox_ReadNavPvt(UBX_NAV_PVT_t* nav);
bool UBlox_GetTimeGPS(UBX_NAV_TIMEGPS_t* timegps);
#endif

