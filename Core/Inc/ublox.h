#ifndef UBLOX_H
#define UBLOX_H

#include "stm32u0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// I2C handle
extern I2C_HandleTypeDef hi2c3;

/* ---------------------------------------------------------------------------
 * DDC stream pump.
 *
 * MIA-M10 streams NMEA on the DDC (I2C) port by default. Register 0xFF is the
 * output stream and the module's register pointer defaults to it, so a plain
 * read returns stream data; 0xFF is also what comes back when the stream is
 * empty, and since NMEA is 7-bit ASCII that byte is an unambiguous idle marker.
 * The 0xFD/0xFE byte-count registers are deliberately not used - see the note
 * in ublox.c for why reading them here does not work.
 *
 * Framing, checksum verification and the sentence queue live in nmea.c.
 * ------------------------------------------------------------------------- */

/* Discard whatever has accumulated. Call once at start-up. */
void UBlox_Init(void);

/* Drain the module. Main loop only - it performs blocking I2C3 transfers. */
void UBlox_Pump(void);

/* Diagnostics, readable from the debugger. */
extern volatile uint32_t ublox_pump_calls;    /* successful blocks read       */
extern volatile uint32_t ublox_bytes_total;   /* real NMEA bytes since boot   */
extern volatile uint32_t ublox_filler_total;  /* 0xFF idle bytes discarded    */
extern volatile uint16_t ublox_last_real;     /* real bytes in the last pump  */
extern volatile uint32_t ublox_err_count;     /* failed I2C3 transfers        */

#endif
