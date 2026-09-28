/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file gem_diag.cpp
 *
 * Diagnostics for the T3 Gemstone O1 (TI AM67, MAIN R5F):
 *  - pmu: Cortex-R5 PMU event counts over the whole system
 *  - mem: access latency of memories and peripheral registers
 *  - spi: time spent in the MCU_MCSPI0 driver
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/module.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/spi/spi.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
	struct am67_mcspi_stats_s {
		uint32_t transfers;
		uint32_t words;
		uint64_t xfer_cycles;
		uint64_t select_cycles;
		uint64_t config_cycles;
		uint32_t max_xfer_cycles;
		uint32_t max_xfer_words;
		uint32_t fifo_stalls;
		uint32_t eot_timeouts;
		uint32_t fail_stat;
		uint32_t fail_rx;
	};

	void am67_mcspi_stats(struct spi_dev_s *dev, struct am67_mcspi_stats_s *stats, bool reset);
	void am67_mcspi_board_select(struct spi_dev_s *dev, uint8_t channel, bool selected);
	int am67_mcspi_take_errors(struct spi_dev_s *dev);
}

extern "C" __EXPORT int gem_diag_main(int argc, char *argv[]);

namespace
{

constexpr float CPU_MHZ = 800.f;

inline uint32_t ccnt()
{
	uint32_t v;
	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(v));
	return v;
}

inline void pmu_select(uint32_t idx)
{
	asm volatile("mcr p15, 0, %0, c9, c12, 5" :: "r"(idx));
	asm volatile("isb");
}

inline void pmu_set_event(uint32_t idx, uint32_t event)
{
	pmu_select(idx);
	asm volatile("mcr p15, 0, %0, c9, c13, 1" :: "r"(event));
}

inline uint32_t pmu_read(uint32_t idx)
{
	uint32_t v;
	pmu_select(idx);
	asm volatile("mrc p15, 0, %0, c9, c13, 2" : "=r"(v));
	return v;
}

inline void pmu_reset_events()
{
	uint32_t pmcr;
	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r"(pmcr));
	pmcr |= (1u << 0) | (1u << 1); // E, P (event counters only; PMCCNTR untouched)
	asm volatile("mcr p15, 0, %0, c9, c12, 0" :: "r"(pmcr));
	asm volatile("mcr p15, 0, %0, c9, c12, 1" :: "r"(0x7u)); // PMCNTENSET counters 0-2
	asm volatile("isb");
}

struct Event {
	uint8_t code;
	const char *name;
};

// Cortex-R5 TRM, "Event type and event counter registers"
constexpr Event events[][3] = {
	{{0x11, "cycles (check vs CCNT)"}, {0x08, "instructions"}, {0x01, "I-cache miss"}},
	{{0x03, "D-cache miss"}, {0x04, "D-cache access"}, {0x43, "external mem request"}},
	{{0x40, "stall: instr buffer (cyc)"}, {0x41, "stall: data dependency (cyc)"}, {0x44, "stall: LSU busy (cyc)"}},
	{{0x47, "IRQ disabled (cyc)"}, {0x46, "FIQ disabled (cyc)"}, {0x10, "branch mispredict"}},
	{{0x42, "D-cache write-back"}, {0x45, "store buffer drain"}, {0x0a, "exceptions taken"}},
};

int pmu(unsigned ms)
{
	PX4_INFO("PMU, %u ms per group (whole system, idle loop included)", ms);

	for (const auto &group : events) {
		for (uint32_t i = 0; i < 3; i++) {
			pmu_set_event(i, group[i].code);
		}

		pmu_reset_events();
		const uint32_t c0 = ccnt();
		usleep(ms * 1000);
		uint32_t v[3];

		for (uint32_t i = 0; i < 3; i++) {
			v[i] = pmu_read(i);
		}

		const uint32_t cycles = ccnt() - c0;

		for (uint32_t i = 0; i < 3; i++) {
			printf("  0x%02x %-30s %10lu  %7.3f%% of cycles  %9.0f /s\n", group[i].code, group[i].name,
			       (unsigned long)v[i], (double)(100.f * v[i] / cycles), (double)(v[i] * 1000.f / ms));
		}
	}

	return 0;
}

// Average cycles of one volatile 32-bit read of addr, IRQs masked (FIQ still on).
float time_read(uintptr_t addr, unsigned n)
{
	asm volatile("" : "+r"(addr)); // hide the constant address from -Warray-bounds
	volatile uint32_t *p = (volatile uint32_t *)addr;
	irqstate_t flags = px4_enter_critical_section();
	(void)*p;
	const uint32_t t0 = ccnt();

	for (unsigned i = 0; i < n; i++) {
		(void)*p;
	}

	const uint32_t t1 = ccnt();
	px4_leave_critical_section(flags);
	return (float)(t1 - t0) / n;
}

float time_write(uintptr_t addr, uint32_t value, unsigned n)
{
	asm volatile("" : "+r"(addr));
	volatile uint32_t *p = (volatile uint32_t *)addr;
	irqstate_t flags = px4_enter_critical_section();
	const uint32_t t0 = ccnt();

	for (unsigned i = 0; i < n; i++) {
		*p = value;
	}

	asm volatile("dsb");
	const uint32_t t1 = ccnt();
	px4_leave_critical_section(flags);
	return (float)(t1 - t0) / n;
}

// One read per 32-byte line over a buffer larger than the 32 KB D-cache.
float time_ddr_miss(size_t bytes)
{
	uint32_t *buf = (uint32_t *)malloc(bytes);

	if (buf == nullptr) {
		return -1.f;
	}

	memset(buf, 1, bytes);
	const unsigned lines = bytes / 32;
	uint32_t sum = 0;
	uint32_t best = UINT32_MAX;

	for (int pass = 0; pass < 3; pass++) {
		irqstate_t flags = px4_enter_critical_section();
		const uint32_t t0 = ccnt();

		for (unsigned i = 0; i < lines; i++) {
			sum += ((volatile uint32_t *)buf)[i * 8];
		}

		const uint32_t t = ccnt() - t0;
		px4_leave_critical_section(flags);

		if (pass > 0 && t < best) {
			best = t;
		}
	}

	free(buf);
	(void)sum;
	return (float)best / lines;
}

void print_latency(const char *name, float cycles)
{
	printf("  %-38s %8.1f cyc  %8.1f ns\n", name, (double)cycles, (double)(cycles * 1000.f / CPU_MHZ));
}

int mem()
{
	static uint32_t cached_word;
	uint32_t stack_word = 0;
	constexpr unsigned N = 200;

	PX4_INFO("read latency, %u reads each, loop overhead included", N);
	print_latency("ATCM 0x00000100", time_read(0x100, N));
	print_latency("BTCM 0x41010000", time_read(0x41010000, N));
	print_latency("task stack (DDR, cached)", time_read((uintptr_t)&stack_word, N));
	print_latency("DDR cached, hit", time_read((uintptr_t)&cached_word, N));
	print_latency("DDR cached, miss (256 KB stride 32)", time_ddr_miss(256 * 1024));
	print_latency("DDR non-cacheable 0xA2100000", time_read(0xa2100000, N));
	print_latency("VIM (R5F local) 0x2FFF0000", time_read(0x2fff0000, N));
	print_latency("MAIN DMTimer1 TCRR 0x0241003C", time_read(0x0241003c, N));
	print_latency("MAIN UART6 (GPS) SPR 0x0286001C", time_read(0x0286001c, N));
	print_latency("MAIN EPWM0 TBCTR 0x23000008", time_read(0x23000008, N));
	print_latency("MCU MCSPI0 REVISION 0x04B00100", time_read(0x04b00100, N));
	print_latency("MCU MCSPI0 CH3STAT 0x04B0016C", time_read(0x04b00130 + 3 * 0x14, N));

	PX4_INFO("write latency (strongly-ordered register region)");
	const uint32_t spr = *(volatile uint32_t *)0x0286001c;
	print_latency("MAIN UART6 SPR write", time_write(0x0286001c, spr, N));
	print_latency("DDR cached write", time_write((uintptr_t)&cached_word, 1, N));
	return 0;
}

int spi(unsigned ms)
{
	struct spi_dev_s *dev = px4_spibus_initialize(1);

	if (dev == nullptr) {
		PX4_ERR("no SPI bus 1");
		return 1;
	}

	am67_mcspi_stats_s s{};
	am67_mcspi_stats(dev, &s, true);
	usleep(ms * 1000);
	am67_mcspi_stats(dev, &s, true);

	printf("  FIFO stalls %lu, EOT timeouts %lu, last failure CHSTAT 0x%08lx after %lu words\n",
	       (unsigned long)s.fifo_stalls, (unsigned long)s.eot_timeouts, (unsigned long)s.fail_stat,
	       (unsigned long)s.fail_rx);

	if (s.transfers == 0) {
		PX4_INFO("no transfers");
		return 0;
	}

	const float us = 1.f / CPU_MHZ;
	const float t_xfer = s.xfer_cycles * us;
	const float t_sel = s.select_cycles * us;
	const float t_cfg = s.config_cycles * us;
	const float total = t_xfer + t_sel + t_cfg;
	PX4_INFO("MCU_MCSPI0 over %u ms: %lu transfers, %lu words", ms, (unsigned long)s.transfers,
		 (unsigned long)s.words);
	printf("  driver busy        %9.0f us  (%.1f%% CPU)\n", (double)total, (double)(total / (ms * 10.f)));
	printf("  exchange           %9.0f us  %.2f us/word\n", (double)t_xfer, (double)(t_xfer / s.words));
	printf("  select+deselect    %9.0f us  %.2f us/transfer\n", (double)t_sel, (double)(t_sel / s.transfers));
	printf("  set freq/mode/bits %9.0f us  %.2f us/transfer\n", (double)t_cfg, (double)(t_cfg / s.transfers));
	printf("  longest exchange   %9.1f us  (%lu words)\n", (double)(s.max_xfer_cycles * us),
	       (unsigned long)s.max_xfer_words);
	return 0;
}

// Read ICM-20948 WHO_AM_I (0xEA) on MCSPI channel 3, n bytes long.
int spi_whoami(unsigned n)
{
	struct spi_dev_s *dev = px4_spibus_initialize(1);
	uint8_t tx[64] {};
	uint8_t rx[64] {};

	if (dev == nullptr || n < 2 || n > sizeof(tx)) {
		return 1;
	}

	tx[0] = 0x80; // WHO_AM_I | read, bank 0 assumed
	SPI_LOCK(dev, true);
	SPI_SETFREQUENCY(dev, 7000000);
	SPI_SETMODE(dev, SPIDEV_MODE3);
	SPI_SETBITS(dev, 8);
	am67_mcspi_board_select(dev, 3, true);
	SPI_EXCHANGE(dev, tx, rx, n);
	am67_mcspi_board_select(dev, 3, false);
	const int errors = am67_mcspi_take_errors(dev);
	SPI_LOCK(dev, false);

	printf("errors %d, rx:", errors);

	for (unsigned i = 0; i < n; i++) {
		printf(" %02x", rx[i]);
	}

	printf("\n");
	am67_mcspi_stats_s s{};
	am67_mcspi_stats(dev, &s, false);
	printf("FIFO stalls %lu, EOT timeouts %lu, CHSTAT 0x%08lx after %lu words\n",
	       (unsigned long)s.fifo_stalls, (unsigned long)s.eot_timeouts, (unsigned long)s.fail_stat,
	       (unsigned long)s.fail_rx);
	return 0;
}

// PC sampling profiler: an HRT callout (in the HRT interrupt) records the
// PC of the interrupted context into 32-byte buckets. Code that runs with
// IRQs masked (ISRs, critical sections) is not sampled.
struct Bucket {
	uint32_t key;
	uint32_t count;
};

constexpr unsigned PROF_BITS = 13;
constexpr unsigned PROF_SIZE = 1u << PROF_BITS;
Bucket *g_hist;
volatile uint32_t g_samples;
volatile uint32_t g_dropped;
hrt_call g_prof_call;

void prof_sample(void *)
{
	const uint32_t key = (up_getusrpc(NULL) >> 5) | 0x80000000u; // never 0
	unsigned h = (key * 2654435761u) >> (32 - PROF_BITS);

	for (unsigned i = 0; i < 32; i++) {
		Bucket &b = g_hist[(h + i) & (PROF_SIZE - 1)];

		if (b.key == key || b.key == 0) {
			b.key = key;
			b.count++;
			g_samples = g_samples + 1;
			return;
		}
	}

	g_dropped = g_dropped + 1;
}

int prof(unsigned seconds, unsigned top)
{
	g_hist = (Bucket *)calloc(PROF_SIZE, sizeof(Bucket));

	if (g_hist == nullptr) {
		return 1;
	}

	g_samples = 0;
	g_dropped = 0;
	memset(&g_prof_call, 0, sizeof(g_prof_call));
	hrt_call_every(&g_prof_call, 1000, 101, prof_sample, nullptr); // ~9.9 kHz, off the 2.5 ms loop
	sleep(seconds);
	hrt_cancel(&g_prof_call);

	PX4_INFO("%lu samples, %lu dropped; top %u 32-byte blocks (addr count permille):",
		 (unsigned long)g_samples, (unsigned long)g_dropped, top);

	for (unsigned n = 0; n < top; n++) {
		unsigned best = PROF_SIZE;

		for (unsigned i = 0; i < PROF_SIZE; i++) {
			if (g_hist[i].count != 0 && (best == PROF_SIZE || g_hist[i].count > g_hist[best].count)) {
				best = i;
			}
		}

		if (best == PROF_SIZE) {
			break;
		}

		printf("P %08lx %lu %lu\n", (unsigned long)(g_hist[best].key << 5), (unsigned long)g_hist[best].count,
		       (unsigned long)(1000ull * g_hist[best].count / g_samples));
		g_hist[best].count = 0;
	}

	free(g_hist);
	g_hist = nullptr;
	return 0;
}

int usage()
{
	PRINT_MODULE_DESCRIPTION("T3 Gemstone O1 (AM67 R5F) diagnostics.");
	PRINT_MODULE_USAGE_NAME_SIMPLE("gem_diag", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pmu", "PMU event counts");
	PRINT_MODULE_USAGE_ARG("<ms>", "window per event group (default 1000)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("mem", "memory and register read/write latency");
	PRINT_MODULE_USAGE_COMMAND_DESCR("prof", "PC sampling profile: <seconds> <top blocks>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("spi", "MCU_MCSPI0 driver time");
	PRINT_MODULE_USAGE_ARG("<ms>", "window (default 2000)", true);
	return 1;
}

} // namespace

int gem_diag_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	const unsigned ms = (argc > 2) ? (unsigned)atoi(argv[2]) : 0;

	if (!strcmp(argv[1], "pmu")) {
		return pmu(ms > 0 && ms < 5000 ? ms : 1000);

	} else if (!strcmp(argv[1], "mem")) {
		return mem();

	} else if (!strcmp(argv[1], "prof")) {
		return prof(ms > 0 && ms <= 60 ? ms : 10, (argc > 3) ? (unsigned)atoi(argv[3]) : 80);

	} else if (!strcmp(argv[1], "whoami")) {
		return spi_whoami(ms > 0 ? ms : 2);

	} else if (!strcmp(argv[1], "spi")) {
		return spi(ms > 0 ? ms : 2000);
	}

	return usage();
}
