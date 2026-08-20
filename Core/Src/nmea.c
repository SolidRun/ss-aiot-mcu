/*
 * nmea.c
 *
 *  Created on: Aug 20, 2026
 *      Author: ruth
 */


#include "nmea.h"
#include <string.h>

/* Reclaiming the oldest slot has to be atomic against the I2C2 interrupt.
 * Stubbed out on a host build so the framer stays unit-testable off-target. */
#ifdef __arm__
#include "stm32u0xx.h"
#else
static inline uint32_t __get_PRIMASK(void)       { return 0U; }
static inline void     __disable_irq(void)       { }
static inline void     __set_PRIMASK(uint32_t p) { (void)p; }
#endif

/* ---------------------------------------------------------------------------
 * NMEA line framer and sentence queue.
 *
 * Bytes arrive from the GNSS in blocks that may start mid-sentence, so framing
 * always resynchronises on '$'. Every complete, checksum-valid sentence reaches
 * the queue - no type filtering. The SOM runs gpsd, which frames and filters
 * far better than we would, so there is nothing to gain by deciding for it.
 *
 * The one thing the framer does keep for itself is the UTC out of RMC, so that
 * the MCU's RTC does not depend on the SOM being alive to tell it the time.
 *
 * The queue holds whole sentences rather than a byte stream, which means that
 * when it overflows an entire sentence is dropped and never half of one - the
 * consumer never sees a truncated line.
 *
 * Single producer (main loop, NMEA_Feed) and single consumer (I2C2 interrupt,
 * NMEA_Pop). The producer only advances q_head, the consumer only q_head's
 * counterpart q_tail and its own cursor, so no locking is needed on Cortex-M0+.
 * ------------------------------------------------------------------------- */

typedef struct {
    uint8_t buf[NMEA_MAX_SENTENCE];
    uint8_t len;
} nmea_slot_t;

static nmea_slot_t      slots[NMEA_SLOTS];
static volatile uint8_t q_head;      /* producer advances */
static volatile uint8_t q_tail;      /* consumer advances */
static uint8_t          q_cursor;    /* consumer only: offset into slots[q_tail] */

static uint8_t  line[NMEA_MAX_SENTENCE];
static uint16_t line_len;
static bool     in_sentence;

volatile uint32_t nmea_accepted;
volatile uint32_t nmea_bad_csum;
volatile uint32_t nmea_dropped_old;
volatile uint32_t nmea_torn;
volatile uint32_t nmea_bytes_out;
volatile uint32_t nmea_pop_empty;
volatile uint32_t nmea_overlong;
volatile uint32_t nmea_time_updates;
volatile uint32_t nmea_time_nofix;

static nmea_time_t   last_time;
static volatile bool time_fresh;

static int8_t hexval(uint8_t c)
{
    if (c >= '0' && c <= '9') { return (int8_t)(c - '0'); }
    if (c >= 'A' && c <= 'F') { return (int8_t)(c - 'A' + 10); }
    if (c >= 'a' && c <= 'f') { return (int8_t)(c - 'a' + 10); }
    return -1;
}

/* Comma/star separated field n of the current line, NULL when absent.
 * Field 0 is the "$xxRMC" talker+type itself. */
static const uint8_t *field(uint8_t n, uint16_t *flen)
{
    uint16_t i;
    uint16_t f = 0;
    uint16_t start = 0;

    for (i = 0; i < line_len; i++) {
        if ((line[i] == (uint8_t)',') || (line[i] == (uint8_t)'*')) {
            if (f == n) {
                *flen = i - start;
                return &line[start];
            }
            f++;
            start = (uint16_t)(i + 1U);
        }
    }
    return NULL;
}

static bool all_digits(const uint8_t *p, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++) {
        if ((p[i] < (uint8_t)'0') || (p[i] > (uint8_t)'9')) {
            return false;
        }
    }
    return true;
}

static uint8_t two(const uint8_t *p)
{
    return (uint8_t)(((p[0] - (uint8_t)'0') * 10U) + (p[1] - (uint8_t)'0'));
}

/* $xxRMC,hhmmss.ss,A,lat,N,lon,E,speed,course,ddmmyy,...
 *         field 1  f2                    field 9                       */
static void take_rmc_time(void)
{
    const uint8_t *t;
    const uint8_t *s;
    const uint8_t *d;
    uint16_t tl = 0, sl = 0, dl = 0;
    nmea_time_t v;

    if (line_len < 6U) { return; }
    if (!((line[3] == 'R') && (line[4] == 'M') && (line[5] == 'C'))) { return; }

    t = field(1U, &tl);
    s = field(2U, &sl);
    d = field(9U, &dl);
    if ((t == NULL) || (s == NULL) || (d == NULL)) { return; }

    /* A populated time and date field is the validity signal, and status is
     * deliberately NOT part of the test.
     *
     * RMC status tracks the POSITION fix; it is 'V' whenever there is no fix,
     * even when the receiver knows the time perfectly well. The M10 emits time
     * and date only when it considers them valid: CFG-NMEA-OUT_INVTIME and
     * CFG-NMEA-OUT_INVDATE default to false, so an invalid time or date comes
     * out as an EMPTY field. u-blox document this with the example
     *
     *     $GPGLL,,,,,124924.00,V,N*42     <- invalid position, VALID time
     *     $GPGLL,,,,,,V,N*64              <- time unknown (cold start)
     *
     * so "both fields present and well formed" is exactly equivalent to
     * validDate && validTime in UBX-NAV-PVT - which is what the old UBX path
     * tested with a mask of 0x03 on that field. Requiring status 'A' as well
     * threw away every fix-less-but-time-valid epoch, which indoors is all of
     * them.
     *
     * Note that validDate can lag validTime, so an RMC with a populated time
     * and an empty date is a legitimate state - both are required here. */
    if ((tl < 6U) || (dl < 6U))                   { return; }
    if (!all_digits(t, 6U) || !all_digits(d, 6U)) { return; }

    v.hour  = two(&t[0]);
    v.min   = two(&t[2]);
    v.sec   = two(&t[4]);
    v.day   = two(&d[0]);
    v.month = two(&d[2]);
    v.year  = two(&d[4]);

    /* refuse to hand the RTC something impossible */
    if ((v.hour > 23U) || (v.min > 59U) || (v.sec > 59U)) { return; }
    if ((v.month < 1U) || (v.month > 12U))                { return; }
    if ((v.day   < 1U) || (v.day   > 31U))                { return; }

    last_time  = v;
    time_fresh = true;
    nmea_time_updates++;
    if ((sl < 1U) || (s[0] != (uint8_t)'A')) {
        nmea_time_nofix++;      /* time without a position fix - accepted, but
                                 * worth seeing separately: fullyResolved is
                                 * less likely, so a whole-second error is
                                 * more likely than with a fix. */
    }
}

static void push(void)
{
    uint8_t next = (uint8_t)((q_head + 1U) % NMEA_SLOTS);

    if (next == q_tail) {
        /* Full. Reclaim the oldest slot so the queue always holds the newest
         * NMEA_SLOTS-1 sentences. A master that polls occasionally has to get
         * fresh data - not whatever happened to be in the buffer at the moment
         * it first filled up, which is what dropping the newest would give it.
         *
         * The consumer runs in the I2C2 interrupt and can preempt this, so the
         * reclaim masks interrupts. If the consumer was part-way through the
         * slot being reclaimed its cursor is reset and it resumes at the next
         * sentence; the master then sees one truncated line, which fails its
         * own NMEA checksum and is discarded there. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (q_cursor != 0U) {
            nmea_torn++;
        }
        q_tail   = (uint8_t)((q_tail + 1U) % NMEA_SLOTS);
        q_cursor = 0;
        __set_PRIMASK(primask);
        nmea_dropped_old++;
    }
    memcpy(slots[q_head].buf, line, line_len);
    slots[q_head].len = (uint8_t)line_len;
    q_head = next;
    nmea_accepted++;
}

static void finish(void)
{
    uint16_t star = 0;
    uint16_t back = 0;
    uint16_t i;
    int8_t   hi, lo;
    uint8_t  want, got = 0;

    /* shortest useful sentence is "$xxxxx*HH\r\n" */
    if (line_len < 10U) {
        nmea_bad_csum++;
        return;
    }

    /* find the checksum delimiter, searching back over "*HH\r\n" at most */
    for (i = line_len; (i > 0U) && (back < 7U); i--, back++) {
        if (line[i - 1U] == '*') {
            star = i - 1U;
            break;
        }
    }
    if (star == 0U || (star + 2U) >= line_len) {
        nmea_bad_csum++;
        return;
    }

    hi = hexval(line[star + 1U]);
    lo = hexval(line[star + 2U]);
    if (hi < 0 || lo < 0) {
        nmea_bad_csum++;
        return;
    }
    want = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);

    for (i = 1U; i < star; i++) {   /* XOR everything between '$' and '*' */
        got ^= line[i];
    }
    if (got != want) {
        nmea_bad_csum++;
        return;
    }

    take_rmc_time();   /* before push(): needs the line, not the slot */
    push();
}

void NMEA_Reset(void)
{
    q_head = 0;
    q_tail = 0;
    q_cursor = 0;
    line_len = 0;
    in_sentence = false;

    nmea_accepted = 0;
    nmea_bad_csum = 0;
    nmea_dropped_old = 0;
    nmea_torn = 0;
    nmea_bytes_out = 0;
    nmea_pop_empty = 0;
    nmea_overlong = 0;
    nmea_time_updates = 0;
    nmea_time_nofix = 0;
    time_fresh = false;
}

void NMEA_Feed(uint8_t b)
{
    if (b == (uint8_t)'$') {         /* always resynchronise here */
        line_len = 0;
        line[line_len++] = b;
        in_sentence = true;
        return;
    }
    if (!in_sentence) {
        return;                      /* mid-sentence block start: discard */
    }
    if (line_len >= NMEA_MAX_SENTENCE) {
        nmea_overlong++;
        in_sentence = false;
        return;
    }
    line[line_len++] = b;

    if (b == (uint8_t)'\n') {
        finish();
        in_sentence = false;
    }
}

bool NMEA_Pending(void)
{
    return (q_tail != q_head);
}

uint8_t NMEA_Pop(uint8_t *dst, uint8_t max)
{
    nmea_slot_t *s;
    uint8_t      n;

    if (q_tail == q_head) {
        nmea_pop_empty++;       /* the consumer drained the queue: it is keeping
                                 * up. If this never moves while
                                 * nmea_dropped_old climbs, it is not. */
        return 0;
    }
    s = &slots[q_tail];
    n = (uint8_t)(s->len - q_cursor);
    if (n > max) {
        n = max;
    }
    memcpy(dst, &s->buf[q_cursor], n);
    q_cursor = (uint8_t)(q_cursor + n);

    if (q_cursor >= s->len) {
        q_cursor = 0;
        q_tail = (uint8_t)((q_tail + 1U) % NMEA_SLOTS);
    }
    nmea_bytes_out += n;
    return n;
}

bool NMEA_GetTime(nmea_time_t *out)
{
    if (!time_fresh) {
        return false;
    }
    *out = last_time;
    time_fresh = false;
    return true;
}
