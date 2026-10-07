/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hrt.c
 *
 * High-resolution timer for the TI AM67 / J722S Cortex-R5F, backed by a
 * K3 DMTimer.
 *
 * PX4 requires a free-running microsecond clock (hrt_absolute_time()) and a
 * one-shot compare interrupt to dispatch the callout queue (hrt_call_*()).
 * We do NOT use the NuttX DMTimer driver; we claim a spare DMTimer and drive
 * it directly, exactly like the STM32/NXP HRT ports do.
 *
 * Timer selection (see also boards/.../src/board_config.h, which may override):
 *   - DMTimer0 @ 0x02400000 (IRQ 24) is already owned by the NuttX system tick
 *     (arch/arm/src/am67/am67_timer.c), so we use
 *   - DMTimer1 @ 0x02410000 (IRQ 25) for the HRT.
 *
 * IMPORTANT INTEGRATION NOTES (the two things most likely to go wrong):
 *
 *   1. On K3 SoCs every peripheral is power/clock-gated by the Device
 *      Manager (system firmware / TISCI). hrt_tim_init() asks for the timer
 *      itself before the first register access, so its power no longer
 *      depends on Linux or U-Boot, and checks that its functional clock is
 *      HRT_TIMER_RATE. A dead module or a stopped counter is still detected
 *      and LOUDLY logged below.
 *
 *   2. The DMTimer1 base address (0x02410000) is derived from the universal K3
 *      0x10000 instance stride relative to DMTimer0 (0x02400000); it was not
 *      cross-checked against a J722S memory-map header (am67 has none). The
 *      TIDR sanity check below will flag a wrong base as a dead module.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <syslog.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

/****************************************************************************
 * Logging
 *
 * hrt_init() runs very early in boot, before the PX4 logging layer is fully
 * up, so error/warn paths go straight to syslog() (always safe) rather than
 * PX4_ERR(). Info-level tracing is gated behind CONFIG_DEBUG_HRT.
 ****************************************************************************/

#define hrterr(fmt, ...)   syslog(LOG_ERR,     "[hrt] " fmt "\n", ##__VA_ARGS__)
#define hrtwarn(fmt, ...)  syslog(LOG_WARNING, "[hrt] " fmt "\n", ##__VA_ARGS__)
#define hrtnotice(fmt, ...) syslog(LOG_INFO,   "[hrt] " fmt "\n", ##__VA_ARGS__)

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

/****************************************************************************
 * DMTimer selection (board_config.h may override any of these)
 ****************************************************************************/

#ifndef HRT_TIMER_BASE
#  define HRT_TIMER_BASE   0x02410000u   /* MAIN DMTimer1 */
#endif

#ifndef HRT_TIMER_IRQ
#  define HRT_TIMER_IRQ    25            /* CSLR_R5FSS0_CORE0_INTR_TIMER1_INTR_PEND_0 */
#endif

#ifndef HRT_TIMER_RATE
#  define HRT_TIMER_RATE   25000000u     /* DMTimer input = HFOSC0 @ 25 MHz */
#endif

#ifndef HRT_TIMER_TISCI_DEV
#  define HRT_TIMER_TISCI_DEV 37u        /* J722S TISCI device of MAIN DMTimer1 */
#endif

#define HRT_TIMER_TISCI_FCK 2u           /* its functional clock (Linux DTS) */

/* arch/arm/src/am67/am67_tisci.c */
int am67_tisci_device_require(uint32_t id);
int am67_tisci_get_freq(uint32_t dev, uint8_t clk, uint64_t *hz);

/* Ticks per microsecond. 25 MHz -> 25 ticks/us. No power-of-two prescaler
 * yields exactly 1 MHz from 25 MHz, so we run the counter at the full input
 * rate and convert in software (see hrt_absolute_time()).
 */
#define HRT_TICKS_PER_US   (HRT_TIMER_RATE / 1000000u)

#if (HRT_TICKS_PER_US * 1000000u) != HRT_TIMER_RATE
#  error "HRT_TIMER_RATE must be an integer number of MHz"
#endif

/****************************************************************************
 * K3 DMTimer register map (offsets from HRT_TIMER_BASE)
 ****************************************************************************/

#define DMTIMER_TIDR            0x00u  /* Identification register (revision) */
#define DMTIMER_IRQSTATUS       0x28u  /* IRQ status, write-1-to-clear       */
#define DMTIMER_IRQENABLE_SET   0x2Cu  /* IRQ enable, write-1-to-set         */
#define DMTIMER_IRQENABLE_CLR   0x30u  /* IRQ enable, write-1-to-clear       */
#define DMTIMER_TCLR            0x38u  /* Control register                   */
#define DMTIMER_TCRR            0x3Cu  /* Counter register (free-running)    */
#define DMTIMER_TLDR            0x40u  /* Load/reload register               */
#define DMTIMER_TWPS            0x48u  /* Write-posting status               */
#define DMTIMER_TMAR            0x4Cu  /* Match/compare register             */
#define DMTIMER_TSICR           0x54u  /* Interface control (POSTED = bit 2)  */

/* TCLR bits */
#define TCLR_ST                 (1u << 0)  /* Start                          */
#define TCLR_AR                 (1u << 1)  /* Auto-reload                    */
#define TCLR_CE                 (1u << 6)  /* Compare enable                 */

/* IRQ flag bits (IRQSTATUS / IRQENABLE_*) */
#define DMTIMER_IRQ_MAT         (1u << 0)  /* Compare match                  */
#define DMTIMER_IRQ_OVF         (1u << 1)  /* Overflow (counter wrap)        */
#define DMTIMER_IRQ_TCAR        (1u << 2)  /* Capture                        */

#define REG(_off)  (*(volatile uint32_t *)(HRT_TIMER_BASE + (_off)))

/****************************************************************************
 * Callout scheduling parameters (microseconds)
 ****************************************************************************/

/* Maximum time we schedule a compare interrupt ahead. This must be well below
 * the counter period so hrt_absolute_time() runs at least once per counter
 * wrap (2^32 / 25 MHz ~= 171.8 s). 50 ms leaves an enormous margin.
 */
#define HRT_INTERVAL_MAX   50000

/* Minimum time we ever schedule ahead, so a compare deadline is never set in
 * the past (which on a 32-bit counter would otherwise stall for a full 171 s
 * period until the next wrap). */
#define HRT_INTERVAL_MIN   50

/****************************************************************************
 * Private data
 ****************************************************************************/

/* The callout queue, ordered by deadline. */
static struct sq_queue_s callout_queue;

/* Set if init found the module dead (bad TIDR) or not advancing. */
static bool hrt_timer_suspect;

/* Interrupt-latency histogram, exported to the shared hrt_ioctl layer and the
 * `hrt` console command via drv_hrt.h. Thresholds are in microseconds. */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/* Compare value last programmed (ticks) and counter value at interrupt entry
 * (ticks), used to measure how late we serviced the compare. */
static uint32_t latency_baseline;
static uint32_t latency_actual;

/****************************************************************************
 * Private prototypes
 ****************************************************************************/

static void hrt_call_enter(struct hrt_call *entry);
static void hrt_call_reschedule(void);
static void hrt_call_invoke(void);
static void hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline,
			      hrt_abstime interval, hrt_callout callout, void *arg);
static void hrt_latency_update(void);

/****************************************************************************
 * Hardware layer
 ****************************************************************************/

/**
 * Read-back based liveness probe. Returns true if the timer's counter is
 * observed to advance. Used only at init; deliberately avoids any delay
 * primitive (the OS timer may not be running yet) by polling the counter.
 */
static bool hrt_counter_is_advancing(void)
{
	uint32_t first = REG(DMTIMER_TCRR);

	for (volatile uint32_t i = 0; i < 1000000u; i++) {
		if (REG(DMTIMER_TCRR) != first) {
			return true;
		}
	}

	return false;
}

/**
 * Initialise and start the DMTimer as a free-running 32-bit up-counter with
 * a compare interrupt, and validate that it actually came alive.
 */
static void hrt_tim_init(void)
{
	uint32_t tidr;
	uint64_t fck_hz = 0;

	/* Power first: an unpowered module aborts on the first access. Nothing
	 * else keeps DMTimer1 on once Linux leaves it alone (px4-r5f overlay) or
	 * when U-Boot started this core. */
	if (am67_tisci_device_require(HRT_TIMER_TISCI_DEV) < 0) {
		hrterr("MAIN_TIMER%u (TISCI %u) is not powered", (unsigned)(HRT_TIMER_TISCI_DEV - 36u),
		       (unsigned)HRT_TIMER_TISCI_DEV);
		PANIC();
	}

	/* Every time conversion assumes HRT_TIMER_RATE. A different clock would
	 * make the whole system time wrong (EKF, rate loops): refuse to run. */
	if (am67_tisci_get_freq(HRT_TIMER_TISCI_DEV, HRT_TIMER_TISCI_FCK, &fck_hz) == 0) {
		if (fck_hz != HRT_TIMER_RATE) {
			hrterr("DMTimer fck %llu Hz, expected %u Hz", (unsigned long long)fck_hz, (unsigned)HRT_TIMER_RATE);
			PANIC();
		}

	} else {
		hrtwarn("DMTimer fck unknown (TISCI GET_FREQ failed), assuming %u Hz", (unsigned)HRT_TIMER_RATE);
	}

	/* --- Suspect #1/#2: is the module powered/clocked/assigned, and is the
	 * base address correct? A live K3 DMTimer returns a non-zero revision in
	 * TIDR. All-zeros or all-ones almost always means the module is gated off
	 * by the Device Manager, not assigned to this R5F core, or that
	 * HRT_TIMER_BASE is wrong.
	 */
	tidr = REG(DMTIMER_TIDR);

	if (tidr == 0x00000000u || tidr == 0xFFFFFFFFu) {
		hrt_timer_suspect = true;
		hrterr("DMTimer at base 0x%08x looks DEAD (TIDR=0x%08x).",
		       (unsigned)HRT_TIMER_BASE, (unsigned)tidr);
		hrterr("  Likely causes:");
		hrterr("   - Device Manager did not power/clock MAIN_TIMER1 or did");
		hrterr("     not assign it to r5fss0_core0 (check your Linux DT /");
		hrterr("     remoteproc resource + sysfw board config).");
		hrterr("   - CTRLMMR_TIMER1_CLKSEL @ 0x001081b4 not muxed to HFOSC0.");
		hrterr("   - Wrong HRT_TIMER_BASE (expected DMTimer1 = 0x02410000,");
		hrterr("     DMTimer0/system-tick = 0x02400000). Verify vs J722S TRM.");
		/* Continue: the counter checks below add more evidence, and PX4
		 * higher layers will fail loudly too. */
	}

	/* Leave posted mode (TSICR.POSTED is 1 after reset). A posted write to a
	 * register that still has a write pending is dropped without notice;
	 * this driver writes TCLR back to back here and TMAR twice in a row when
	 * the ISR reschedules. Losing a TMAR write would stop the compare
	 * interrupt until the counter wraps (172 s), and every scheduled work
	 * queue with it. Wait for pending writes first. */
	for (unsigned spins = 0; REG(DMTIMER_TWPS) != 0 && spins < 100000; spins++) {}

	REG(DMTIMER_TSICR) = 0;

	/* Stop the timer and clear any pending interrupts before (re)configuring. */
	REG(DMTIMER_TCLR) = 0;
	REG(DMTIMER_IRQENABLE_CLR) = DMTIMER_IRQ_MAT | DMTIMER_IRQ_OVF | DMTIMER_IRQ_TCAR;
	REG(DMTIMER_IRQSTATUS) = DMTIMER_IRQ_MAT | DMTIMER_IRQ_OVF | DMTIMER_IRQ_TCAR;

	/* Free-running: reload to 0 on overflow, start counting from 0. */
	REG(DMTIMER_TLDR) = 0;
	REG(DMTIMER_TCRR) = 0;
	REG(DMTIMER_TMAR) = 0;

	/* Enable overflow (wrap tracking) and match (callout) interrupts. */
	REG(DMTIMER_IRQENABLE_SET) = DMTIMER_IRQ_MAT | DMTIMER_IRQ_OVF;

	/* Auto-reload + compare-enable, then start. No prescaler (PTV/PRE = 0). */
	REG(DMTIMER_TCLR) = TCLR_AR | TCLR_CE | TCLR_ST;

	/* --- Suspect #1: even with a plausible TIDR, the counter only advances
	 * if the functional clock is actually running. Prove it moved. */
	if (!hrt_counter_is_advancing()) {
		hrt_timer_suspect = true;
		hrterr("DMTimer at base 0x%08x is NOT COUNTING (TCRR stuck at 0x%08x).",
		       (unsigned)HRT_TIMER_BASE, (unsigned)REG(DMTIMER_TCRR));
		hrterr("  The register block responds but the functional clock is");
		hrterr("  off: the Device Manager likely did not enable/mux the");
		hrterr("  MAIN_TIMER1 clock (CTRLMMR_TIMER1_CLKSEL @ 0x001081b4).");
	}

	if (!hrt_timer_suspect) {
		hrtnotice("bound DMTimer base=0x%08x irq=%d rate=%u Hz (TIDR=0x%08x)",
			  (unsigned)HRT_TIMER_BASE, HRT_TIMER_IRQ,
			  (unsigned)HRT_TIMER_RATE, (unsigned)tidr);
	}
}

/**
 * Timer compare/overflow interrupt: run due callouts and re-arm the next one.
 */
static int hrt_tim_isr(int irq, void *context, void *arg)
{
	/* Snapshot the counter as early as possible for latency measurement. */
	latency_actual = REG(DMTIMER_TCRR);

	uint32_t status = REG(DMTIMER_IRQSTATUS);

	/* Ack everything we saw (write-1-to-clear). */
	REG(DMTIMER_IRQSTATUS) = status;

	/* How late were we relative to the programmed compare? */
	if (status & DMTIMER_IRQ_MAT) {
		hrt_latency_update();
	}

	/* An overflow with no match still updates the software time base via the
	 * hrt_absolute_time() call inside hrt_call_invoke()/reschedule(), so we
	 * don't need to special-case DMTIMER_IRQ_OVF here. */
	hrt_call_invoke();
	hrt_call_reschedule();

	return OK;
}

/****************************************************************************
 * Public clock interface
 ****************************************************************************/

/**
 * Return absolute time in microseconds since the timer was initialised.
 *
 * The hardware counter runs at HRT_TIMER_RATE (25 MHz), so we accumulate a
 * 64-bit microsecond base and carry the sub-microsecond tick remainder to
 * stay drift-free. Only 32-bit divides are used (Cortex-R5F friendly).
 *
 * Invariant: this must be called at least once per counter period (~171 s).
 * The periodic compare/overflow interrupts guarantee that comfortably.
 */
hrt_abstime hrt_absolute_time(void)
{
	static volatile hrt_abstime base_us;
	static volatile uint32_t    last_count;
	static volatile uint32_t    tick_remainder;

	irqstate_t flags = px4_enter_critical_section();

	uint32_t count = REG(DMTIMER_TCRR);

	/* Ticks elapsed since last read; unsigned subtraction is wrap-safe. */
	uint32_t elapsed = count - last_count;
	last_count = count;

	/* Convert elapsed ticks (plus carried remainder) to whole microseconds. */
	uint32_t total = elapsed + tick_remainder;
	base_us       += total / HRT_TICKS_PER_US;
	tick_remainder = total % HRT_TICKS_PER_US;

	hrt_abstime abstime = base_us;

	px4_leave_critical_section(flags);

	return abstime;
}

void hrt_store_absolute_time(volatile hrt_abstime *t)
{
	irqstate_t flags = px4_enter_critical_section();
	*t = hrt_absolute_time();
	px4_leave_critical_section(flags);
}

/****************************************************************************
 * Public callout interface
 ****************************************************************************/

void hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();

	irq_attach(HRT_TIMER_IRQ, hrt_tim_isr, NULL);
	up_enable_irq(HRT_TIMER_IRQ);
}

void hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, hrt_absolute_time() + delay, 0, callout, arg);
}

void hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

void hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval,
		    hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, hrt_absolute_time() + delay, interval, callout, arg);
}

static void hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline,
			      hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	/* If the entry is currently queued, remove it (safe even if uninit'd:
	 * sq_rem() only unlinks if actually found in the queue). */
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period   = interval;
	entry->callout  = callout;
	entry->arg      = arg;

	hrt_call_enter(entry);

	px4_leave_critical_section(flags);
}

bool hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

void hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* Prevent a periodic callout from re-entering if cancelled from itself. */
	entry->period = 0;

	px4_leave_critical_section(flags);
}

static void hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call *call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	hrtinfo("scheduled\n");
}

static void hrt_call_invoke(void)
{
	struct hrt_call *call;
	hrt_abstime deadline;

	while (true) {
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrtinfo("call pop\n");

		deadline = call->deadline;

		/* Zero the deadline, marking the call as invoked. */
		call->deadline = 0;

		if (call->callout) {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		/* Re-enter periodic callouts. */
		if (call->period != 0) {
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next compare interrupt for the head of the queue.
 * Must be called with interrupts disabled.
 */
static void hrt_call_reschedule(void)
{
	hrt_abstime now      = hrt_absolute_time();
	struct hrt_call *next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime deadline = now + HRT_INTERVAL_MAX;

	if (next != NULL) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	/* Program the compare register in ticks, relative to the counter value
	 * read *now*, so the match is always in the future (never the past, which
	 * on a 32-bit counter would otherwise stall a full period). delay_us is
	 * bounded to [HRT_INTERVAL_MIN, HRT_INTERVAL_MAX]. */
	uint32_t delay_us    = (uint32_t)(deadline - now);
	uint32_t delay_ticks = delay_us * HRT_TICKS_PER_US;

	uint32_t compare = REG(DMTIMER_TCRR) + delay_ticks;
	REG(DMTIMER_TMAR) = compare;

	/* Remember the target for latency accounting in the ISR. */
	latency_baseline = compare;

	hrtinfo("schedule in %u us (%u ticks)\n",
		(unsigned)delay_us, (unsigned)delay_ticks);
}

/**
 * Bucket how late the compare interrupt was serviced, in microseconds.
 * (latency_actual and latency_baseline are counter ticks; convert to us.)
 */
static void hrt_latency_update(void)
{
	uint32_t latency_ticks = latency_actual - latency_baseline;
	uint32_t latency = latency_ticks / HRT_TICKS_PER_US;
	unsigned index;

	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* Catch-all bucket at the end. */
	latency_counters[index]++;
}

void hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}
