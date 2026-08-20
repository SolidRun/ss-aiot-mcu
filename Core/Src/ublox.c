#include "ublox.h"
#include "nmea.h"
#include <string.h>

#define UBLOX_ADDR (0x42 << 1)

/* ===========================================================================
 * DDC stream pump
 * ---------------------------------------------------------------------------
 * Additive: nothing above is touched. Once the NMEA path is proven the UBX
 * polling functions, their poll messages and the NAV structs can be removed.
 *
 * No use is made of the 0xFD/0xFE byte-count registers. Reading them needs the
 * register-address write and the data read to be joined by a repeated start;
 * doing it as a separate Transmit() and Receive() puts a STOP in between, the
 * module resets its register pointer, and the two bytes that come back are
 * stream filler rather than a count. That was measured: the count read as 0xFF
 * and the pump then "drained" filler at roughly twenty times the real rate.
 *
 * Instead this reads a fixed block straight from the stream (the register
 * pointer defaults to 0xFF) and discards 0xFF bytes. NMEA is 7-bit ASCII, so
 * 0xFF can never be real data - it is an unambiguous idle marker, and the whole
 * question of pointer semantics goes away.
 *
 * I2C3 carries nothing but the GNSS, so always transferring a full block costs
 * nothing that matters: 64 bytes at 100 kHz is about 6 ms, and at a 20 ms
 * cadence that is a third of an otherwise idle bus, with a 3200 B/s ceiling
 * against roughly 600 B/s of real output.
 * ========================================================================= */

#define UBLOX_CHUNK         64U     /* bytes per I2C3 read              */
#define UBLOX_I2C_TIMEOUT   20U     /* ms - short, never HAL_MAX_DELAY   */
#define UBLOX_FLUSH_GUARD   512U    /* bound the start-up flush loop     */
#define UBLOX_FILLER        0xFFU   /* module idle byte                  */

/* Diagnostics. Only meaningful together: pump_calls counts blocks that were
 * actually read, so at any moment
 *
 *     ublox_pump_calls * UBLOX_CHUNK == ublox_bytes_total + ublox_filler_total
 *
 * exactly. If that stops holding, bytes are being lost somewhere between the
 * I2C read and the classification loop. The start-up flush in UBlox_Init()
 * deliberately stays out of these counters. */
volatile uint32_t ublox_pump_calls;    /* successful blocks read       */
volatile uint32_t ublox_bytes_total;   /* real NMEA bytes since boot   */
volatile uint32_t ublox_filler_total;  /* 0xFF idle bytes discarded    */
volatile uint16_t ublox_last_real;     /* real bytes in the last pump  */
volatile uint32_t ublox_err_count;     /* failed I2C3 transfers        */

/* Read one block of stream. Returns false on I2C error. */
static bool UBlox_ReadBlock(uint8_t *buf)
{
    if (HAL_I2C_Master_Receive(&hi2c3, UBLOX_ADDR, buf, UBLOX_CHUNK,
                               UBLOX_I2C_TIMEOUT) != HAL_OK) {
        ublox_err_count++;
        return false;
    }
    return true;
}

void UBlox_Init(void)
{
    uint8_t  scratch[UBLOX_CHUNK];
    uint16_t guard;

    ublox_pump_calls   = 0;
    ublox_bytes_total  = 0;
    ublox_filler_total = 0;
    ublox_last_real    = 0;
    ublox_err_count    = 0;

    NMEA_Reset();

    /* Drain until a whole block comes back as filler, i.e. the module is empty.
     * Measured 9287 bytes of backlog on a board nothing had ever drained. */
    for (guard = 0; guard < UBLOX_FLUSH_GUARD; guard++) {
        uint16_t i;
        uint16_t nonfiller = 0;

        if (!UBlox_ReadBlock(scratch)) {
            return;
        }
        for (i = 0; i < UBLOX_CHUNK; i++) {
            if (scratch[i] != UBLOX_FILLER) {
                nonfiller++;
            }
        }
        if (nonfiller == 0U) {
            return;                 /* empty */
        }
    }
}

void UBlox_Pump(void)
{
    uint8_t  chunk[UBLOX_CHUNK];
    uint16_t i;
    uint16_t real = 0;

    if (!UBlox_ReadBlock(chunk)) {
        return;
    }
    ublox_pump_calls++;

    for (i = 0; i < UBLOX_CHUNK; i++) {
        if (chunk[i] == UBLOX_FILLER) {
            ublox_filler_total++;
            continue;
        }
        real++;
        NMEA_Feed(chunk[i]);
    }

    ublox_last_real    = real;
    ublox_bytes_total += real;
}
