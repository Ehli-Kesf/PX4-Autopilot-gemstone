/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file dshot.c
 *
 * DShot HAL for the TI AM67 R5F: maps the PX4 up_dshot_* interface onto the
 * NuttX EPWM + FIQ bit engine (arch/arm/src/am67/am67_dshot.c). Output
 * channels 0-3 (EPWM0 A/B, EPWM1 A/B) can run DShot150/300/600; eCAP
 * channels stay PWM. Bidirectional DShot is not implemented yet: the
 * capture queries report no support, so the dshot module does not wait for
 * telemetry.
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_dshot.h>
#include <px4_platform_common/log.h>
#include <perf/perf_counter.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>

/* NuttX arch/arm/src/am67/am67_dshot.h */
extern int am67_dshot_init(uint32_t bitrate, uint32_t channel_mask);
extern void am67_dshot_set(unsigned channel, uint16_t value, bool telemetry);
extern int am67_dshot_trigger(void);
extern void am67_dshot_arm(bool armed);

struct am67_dshot_stats_s {
	uint32_t frames;
	uint32_t overruns;
	uint32_t late;
	uint32_t skipped;
	uint32_t slow;
	uint32_t timeouts;
	uint32_t max_exit_ticks;
	uint32_t max_fiq_cycles;
	uint32_t frame_cycles;
	uint32_t period_ticks;
};

extern void am67_dshot_stats(struct am67_dshot_stats_s *stats);

#define AM67_DSHOT_CHANNEL_MASK 0x0fu

/* Motor output continuity: the time between frames (bench-check) */

static perf_counter_t g_interval_perf;

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	(void)edt_enable;

	if (bdshot_channel_mask != 0) {
		syslog(LOG_WARNING, "[dshot] bidirectional DShot not supported yet, sending plain DShot\n");
	}

	/* EPWM1 runs on EPWM0's time base and the FIQ engine writes both
	 * modules, so DShot needs both groups (PWM_MAIN_TIM0 and PWM_MAIN_TIM1).
	 * With one group in PWM the two drivers would fight over EPWM1: send
	 * nothing instead. */
	const uint32_t dshot_channels = channel_mask & AM67_DSHOT_CHANNEL_MASK;

	if ((dshot_channels & 0x3u) == 0 || (dshot_channels & 0xcu) == 0) {
		PX4_ERR("PWM_MAIN_TIM0 and PWM_MAIN_TIM1 must both be DShot; no DShot output");
		return -EINVAL;
	}

	if (g_interval_perf == NULL) {
		g_interval_perf = perf_alloc(PC_INTERVAL, "dshot: frame interval");
	}

	return am67_dshot_init(dshot_pwm_freq, channel_mask & AM67_DSHOT_CHANNEL_MASK);
}

void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	am67_dshot_set(channel, throttle, telemetry);
}

void up_dshot_trigger(void)
{
	if (am67_dshot_trigger() == 0) {
		perf_count(g_interval_perf);
	}
}

int up_dshot_arm(bool armed)
{
	am67_dshot_arm(armed);
	return 0;
}

void up_bdshot_status(void)
{
	struct am67_dshot_stats_s st;

	am67_dshot_stats(&st);
	printf("frames sent: %lu, refused (previous frame busy): %lu, dropped (FIQ late): %lu, of which a period late: %lu\n",
	       (unsigned long)st.frames, (unsigned long)st.overruns, (unsigned long)st.late, (unsigned long)st.skipped);
	printf("bits written past half a period: %lu, frames stopped (FIQ stalled): %lu\n", (unsigned long)st.slow,
	       (unsigned long)st.timeouts);
	printf("worst FIQ end: %lu of %lu ticks per bit, worst FIQ run: %lu cycles, last frame: %lu cycles\n",
	       (unsigned long)st.max_exit_ticks, (unsigned long)st.period_ticks, (unsigned long)st.max_fiq_cycles,
	       (unsigned long)st.frame_cycles);
	printf("bidirectional DShot: not supported\n");
}

uint16_t up_bdshot_get_ready_mask(void)
{
	return 0;
}

int up_bdshot_num_errors(uint8_t channel)
{
	(void)channel;
	return 0;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	(void)channel;
	(void)erpm;
	return -1;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	(void)channel;
	(void)type;
	(void)value;
	return -1;
}

int up_bdshot_channel_online(uint8_t channel)
{
	(void)channel;
	return 0;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	(void)channel;
	return 0;
}
