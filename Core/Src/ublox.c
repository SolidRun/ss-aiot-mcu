#include "ublox.h"
#include <string.h>

#define UBLOX_ADDR (0x42 << 1)


// Poll NAV-PVT command
static uint8_t UBX_NAV_PVT_POLL[8] = {
    0xB5, 0x62,             // Sync chars
    0x01, 0x07,             // Class=0x01 (NAV), ID=0x07 (PVT)
    0x00, 0x00,             // Length = 0
    0x08, 0x19              // CK_A, CK_B
};

// Poll TIMEUTC
static uint8_t UBX_NAV_TIMEUTC_POLL[] = {
    0xB5, 0x62,       // Sync chars
    0x01, 0x21,       // Class=0x01 (NAV), ID=0x21 (TIMEUTC)
    0x00, 0x00,       // Length = 0
    0x22, 0x67        // CK_A, CK_B (checksum)
};

// CFG-GNSS
static uint8_t UBX_CFG_GNSS[] = {
    0xB5, 0x62,             // Sync chars
    0x06, 0x3E,             // CFG-GNSS
    0x00, 0x00,             // Length = 0
    0x44, 0x6B              // CK_A, CK_B
};


//CFG-MSG
static uint8_t UBX_CFG_MSG_NAV_PVT[11] = {
    0xB5, 0x62,             // Sync chars
    0x06, 0x01,             // CFG-MSG
    0x03, 0x00,             // Length = 3
    0x01, 0x07, 0x01,       // Class=0x01 (NAV), ID=0x07 (PVT), Rate=1
    0x13, 0x51              // CK_A, CK_B
};


static void UBX_Send(uint8_t *msg, uint16_t len) {
    HAL_I2C_Master_Transmit(&hi2c1, UBLOX_ADDR, msg, len, 100);
}

// Simple UBX checksum calculation
static void UBX_CalcChecksum(uint8_t* msg, uint8_t len, uint8_t* ck_a, uint8_t* ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for(uint8_t i = 2; i < len; i++)
    {
        *ck_a += msg[i];
        *ck_b += *ck_a;
    }
}

// Initialize UBlox (nothing for polling)
void UBlox_Init(void)
{
    //UBX_Send(UBX_CFG_GNSS, sizeof(UBX_CFG_GNSS));
    //HAL_Delay(100);
    //UBX_Send(UBX_CFG_MSG_NAV_PVT, sizeof(UBX_CFG_MSG_NAV_PVT));
    //HAL_Delay(100);
}

// Read NAV-PVT message (blocking)
bool UBlox_ReadNavPvt(UBX_NAV_PVT_t* nav)
{
    uint8_t header[UBX_HEADER_SIZE];
    uint8_t payload[92]; // NAV-PVT payload length
    uint8_t checksum[UBX_CHECKSUM_SIZE];

    // Send poll request
    if(HAL_I2C_Master_Transmit(&hi2c1, 0x42<<1, UBX_NAV_PVT_POLL, sizeof(UBX_NAV_PVT_POLL), 100) != HAL_OK)
        return false;

    // Read header
    if(HAL_I2C_Master_Receive(&hi2c1, 0x42<<1, header , UBX_HEADER_SIZE , 100) != HAL_OK)
        return false;

    uint16_t length = header[4] | (header[5] << 8);
    if(length != 92) return false;

    // Read payload
    if(HAL_I2C_Master_Receive(&hi2c1, 0x42<<1, payload, length, 0xFF) != HAL_OK)
        return false;

    // Read checksum
    if(HAL_I2C_Master_Receive(&hi2c1, 0x42<<1, checksum, UBX_CHECKSUM_SIZE, 100) != HAL_OK)
        return false;

    // Copy payload to struct (little-endian)
    memcpy(nav, payload, sizeof(UBX_NAV_PVT_t));

    return true;
}

bool UBlox_GetTimeGPS(UBX_NAV_TIMEGPS_t* timegps) {
    // 1. Send Poll request
    if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_ADDR, (uint8_t*)UBX_NAV_TIMEUTC_POLL,
                                sizeof(UBX_NAV_TIMEUTC_POLL), HAL_MAX_DELAY) != HAL_OK)
        return false;

    // 2. Read 2-byte length prefix
    uint8_t len_bytes[2];
    if (HAL_I2C_Master_Receive(&hi2c1, UBLOX_ADDR, len_bytes, 2, 100) != HAL_OK)
        return false;
    uint16_t msg_len = len_bytes[0] | (len_bytes[1] << 8);

    if (msg_len < 8 + 20 + 2)  // header (6) + payload (20) + checksum (2)
        return false;

    // 3. Read the UBX message
    uint8_t buffer[32]; // 6+20+2 = 28 bytes total
    if (HAL_I2C_Master_Receive(&hi2c1, UBLOX_ADDR, buffer, 30, 100) != HAL_OK)
        return false;

    // 4. Verify sync, class, id
    if (buffer[0] != 0xB5 || buffer[1] != 0x62) return false;
    if (buffer[2] != 0x01 || buffer[3] != 0x21) return false;

    // 5. Copy payload into struct
    //memcpy(timeutc, &buffer[6], sizeof(UBX_NAV_TIMEUTC_t));
    if (msg_len <50) return true;

    return false;
}

