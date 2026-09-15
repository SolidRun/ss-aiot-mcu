// SPDX-License-Identifier: BSD-3-Clause
/*
 * timebase.c
 *
 * LPTIM2's counter is 16 bits, so at 4096 Hz it wraps every 16 s. Widened to
 * 32 bits - 291 hours - by counting the overflows in software and splicing the
 * count onto CNT on every read.
 */

#include "timebase.h"

#include <stdbool.h>

#include "lptim.h"

/* MX_LPTIM2_Init() leaves ARR at the counter's full range, so it runs 0..65535
 * and one overflow is 65536 ticks. */
#define TIMEBASE_WRAP_SHIFT  16U

/* Halfway through the counter's range - see Timebase_Now(). */
#define TIMEBASE_HALF        0x8000U

_Static_assert(LSE_VALUE / 8U == TIMEBASE_HZ,
               "LSE_VALUE and the LPTIM2 prescaler disagree with TIMEBASE_HZ");

/* High half of the tick count: overflows of CNT. Written only by the
 * update-event interrupt. */
static volatile uint32_t timebase_wraps;

void Timebase_Init(void)
{
    /* TIMEBASE_HZ holds only for this prescaler, which is CubeMX-generated and
     * can change on a regeneration. */
    if (hlptim2.Init.Clock.Prescaler != LPTIM_PRESCALER_DIV8)
        Error_Handler();

    timebase_wraps = 0;

    /* Enables ARROK, ARRM and REPOK besides the update event. Their callbacks
     * are the HAL's empty ones, and narrowing DIER afterwards would need
     * another DIEROK handshake. */
    if (HAL_LPTIM_Counter_Start_IT(&hlptim2) != HAL_OK)
        Error_Handler();
}

/*
 * The update event is raised by the overflow itself. The autoreload match is
 * raised one tick earlier, when CNT reaches ARR, which would leave a tick in
 * which the high half had advanced while CNT still read 65535 - a composed
 * value one whole wrap too high, for 244us out of every 16 s.
 *
 * One interrupt every 16 s.
 */
void HAL_LPTIM_UpdateEventCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim->Instance == LPTIM2)
        timebase_wraps++;
}

/* LSE is asynchronous to the core clock, so a single read of CNT can catch the
 * counter mid-carry. RM0503 requires reading until two reads agree. */
static uint16_t Timebase_ReadCounter(void)
{
    uint16_t first, second;

    do {
        first  = (uint16_t)LPTIM2->CNT;
        second = (uint16_t)LPTIM2->CNT;
    } while (first != second);

    return second;
}

uint32_t Timebase_Now(void)
{
    uint32_t wraps;
    uint16_t cnt;
    bool     pending;

    /* The interrupt moves the high half, so a run of it during these reads
     * leaves the pair inconsistent. */
    do {
        wraps   = timebase_wraps;
        cnt     = Timebase_ReadCounter();
        pending = __HAL_LPTIM_GET_FLAG(&hlptim2, LPTIM_FLAG_UPDATE) != 0U;
    } while (wraps != timebase_wraps);

    /*
     * A pending update event is an overflow not yet counted, either because
     * the interrupt has not run or because CNT wrapped between the two reads
     * above. CNT decides which epoch it belongs to: the bottom half of the
     * range is past the overflow, the top half has not reached it.
     *
     * The flag is read after CNT so that the race between the two costs one
     * tick rather than one wrap.
     *
     * Holds while the interrupt is serviced within a wrap period. It runs at
     * priority 0 and has 16 s, so only a masked-interrupt region that long
     * would break it.
     */
    if (pending && (cnt < TIMEBASE_HALF))
        wraps++;

    return (wraps << TIMEBASE_WRAP_SHIFT) | cnt;
}
