#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>
#include <stdbool.h>

/* NMEA 0183 caps a sentence at 82 bytes including CRLF; a little slack on top. */
#define NMEA_MAX_SENTENCE   84U
/* The module emits its whole batch once a second - about ten sentences with a
 * fix - so the queue is sized to hold a full second even if the SOM goes quiet
 * for that long. 12 x 84 is roughly 1 kB out of 12 kB. */
#define NMEA_SLOTS          12U

/* UTC taken from a checksum-valid RMC with a fix. */
typedef struct {
    uint8_t year;    /* 00-99, years since 2000 */
    uint8_t month;   /* 1-12  */
    uint8_t day;     /* 1-31  */
    uint8_t hour;    /* 0-23  */
    uint8_t min;     /* 0-59  */
    uint8_t sec;     /* 0-59  */
} nmea_time_t;

/* Producer side - call from the main loop, one byte at a time. */
void NMEA_Reset(void);
void NMEA_Feed(uint8_t b);

/* Consumer side - safe to call from the I2C2 interrupt: pure memory, no I2C.
 * Copies up to max bytes of the oldest pending sentence and consumes them.
 * Returns 0 when nothing is pending. Repeat until it returns 0. */
uint8_t NMEA_Pop(uint8_t *dst, uint8_t max);
bool    NMEA_Pending(void);

/* Time side. Returns true and fills out when a fresh time has arrived since the
 * last call, then clears the fresh flag. Independent of the queue, so the SOM
 * draining or not draining makes no difference to the clock. */
bool NMEA_GetTime(nmea_time_t *out);

/* Diagnostics. */
extern volatile uint32_t nmea_accepted;       /* sentences queued            */
extern volatile uint32_t nmea_bad_csum;       /* checksum or framing failure */
extern volatile uint32_t nmea_time_updates;   /* valid RMC times extracted   */
extern volatile uint32_t nmea_time_nofix;     /* of those, with no fix yet   */
extern volatile uint32_t nmea_dropped_old;    /* oldest discarded for room   */
extern volatile uint32_t nmea_torn;           /* a read was cut short by that */
extern volatile uint32_t nmea_bytes_out;      /* bytes handed to the master   */
extern volatile uint32_t nmea_pop_empty;      /* times the queue ran dry      */
extern volatile uint32_t nmea_overlong;       /* line exceeded the limit     */

#endif
