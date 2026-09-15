// SPDX-License-Identifier: BSD-3-Clause
#include "ublox.h"
#include "nmea.h"
#include "i2c.h"
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
volatile uint32_t ublox_resets;        /* I2C3 recoveries performed    */
volatile uint32_t ublox_reset_errors;  /* recoveries that failed       */

/* Shortest gap between two recoveries, in ms.
 *
 * If a reset did not take, repeating it at the pump's own cadence would spend
 * the main loop on recoveries and nothing else. */
#define UBLOX_RESET_BACKOFF_MS  200U

/* Recover a stuck I2C3.
 *
 * Clearing PE is this peripheral's documented software reset: it returns the
 * state machine and status bits to their reset values and releases SCL and SDA.
 * That covers the MCU holding the bus. It cannot help if the module itself is
 * holding a line down.
 *
 * Not MX_I2C3_Init(), because all three of its failure paths end in
 * Error_Handler(), which disables interrupts and loops forever. A glitch on the
 * GNSS bus must not be able to stop the controller; the SOM would read that as
 * a dead MCU. Re-init here and count a failure instead, to be retried on the
 * next block.
 *
 * hi2c3.Init survives HAL_I2C_DeInit(), so only the two filters need setting
 * again. They are what MX_I2C3_Init() configures beyond HAL_I2C_Init(). */
static void UBlox_ResetBus(void)
{
    ublox_resets++;

    if ((HAL_I2C_DeInit(&hi2c3) != HAL_OK) ||
        (HAL_I2C_Init(&hi2c3) != HAL_OK) ||
        (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) ||
        (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)) {
        ublox_reset_errors++;
    }
}

/* Read one block of stream. Returns false on I2C error. */
static bool UBlox_ReadBlock(uint8_t *buf)
{
    static uint32_t last_reset;
    static bool     reset_since_ok;

    if (HAL_I2C_Master_Receive(&hi2c3, UBLOX_ADDR, buf, UBLOX_CHUNK,
                               UBLOX_I2C_TIMEOUT) == HAL_OK) {
        reset_since_ok = false;
        return true;
    }

    ublox_err_count++;

    /* What failed decides whether a recovery is needed. HAL clears ErrorCode at
     * the start of every transfer, so this describes this block only.
     *
     * AF means the module did not acknowledge: asleep, busy, or absent. HAL's
     * NACK path already sends a STOP and clears CR2, so the peripheral is clean
     * and the next block can just try again.
     *
     * TIMEOUT is the one that leaves damage. HAL abandons the transfer without
     * a STOP and without clearing CR2, so BUSY stays asserted and every later
     * transfer fails the same way, permanently. BERR and ARLO leave the bus
     * undefined as well. Only these three are worth a reset. */
    if ((hi2c3.ErrorCode & (HAL_I2C_ERROR_TIMEOUT |
                            HAL_I2C_ERROR_BERR |
                            HAL_I2C_ERROR_ARLO)) == 0U)
        return false;

    /* Unsigned difference, so the tick wrap at 49.7 days is safe. */
    if (reset_since_ok && ((HAL_GetTick() - last_reset) < UBLOX_RESET_BACKOFF_MS))
        return false;

    last_reset     = HAL_GetTick();
    reset_since_ok = true;
    UBlox_ResetBus();

    return false;
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
    ublox_resets       = 0;
    ublox_reset_errors = 0;

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
