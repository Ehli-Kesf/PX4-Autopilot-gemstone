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
 * @file pwm_servo.c
 *
 * AM67 PX4 PWM servo output layer. Implements the portable up_pwm_servo_* API
 * (drv_pwm_output.h) that the pwm_out module uses, by driving the existing
 * NuttX am67 EPWM/eCAP PWM lower-halves via their pwm_lowerhalf_s ops. All the
 * divider/duty/park/ignition logic is reused from arch/arm/src/am67/am67_pwm.c
 * and am67_ecap.c (copper-verified), so no register code is duplicated here.
 *
 * Channel/group map (see boards/.../src/timer_config.cpp):
 *   group 0 = EPWM0 -> ch 0 (A=NuttX chan 1), ch 1 (B=NuttX chan 2)
 *   group 1 = EPWM1 -> ch 2 (A),              ch 3 (B)
 *   group 2 = eCAP1 -> ch 4 (APWM = NuttX chan 1)
 *
 * NOTE: ch 4 uses eCAP1 (HAT GPIO-16 / A25, MCASP0_AXR3 mode 5), NOT eCAP0:
 * eCAP0's only HAT output is C20 (GPIO-12) mode 3, which collides with
 * EPWM0_B (C20 mode 2) already used by ch 1. eCAP1's pad is free.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/pwm.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

/* NuttX am67 arch entry points (private arch headers are not on the PX4
 * include path, so declare the minimal signatures here; C linkage). */
__BEGIN_DECLS
int am67_epwm_init(void);
struct pwm_lowerhalf_s *am67_epwminitialize(int pwm);
int am67_ecap_init(void);
struct pwm_lowerhalf_s *am67_ecapinitialize(int ecap);
int am67_tisci_device_on(uint32_t id);
__END_DECLS

#define AM67_PWM_NGROUPS   BOARD_NUM_IO_TIMERS      /* 3: EPWM0, EPWM1, eCAP0 */
#define AM67_PWM_NCHANNELS DIRECT_PWM_OUTPUT_CHANNELS /* 5 */

/* Per-group NuttX lower-half handle (index 0=EPWM0, 1=EPWM1, 2=eCAP0). */
static struct pwm_lowerhalf_s *g_lower[AM67_PWM_NGROUPS];
static uint32_t g_rate[AM67_PWM_NGROUPS]  = { 50, 50, 50 }; /* Hz per group */
static uint16_t g_pulse_us[AM67_PWM_NCHANNELS];             /* commanded width */
static bool     g_armed;
static bool     g_inited;
static bool     g_epwm_ok = true;

/* Channel -> group and NuttX in-module channel (1=A, 2=B; eCAP APWM = 1). */
static inline unsigned chan_group(unsigned ch) { return (ch < 4u) ? (ch / 2u) : 2u; }
static inline int8_t   chan_local(unsigned ch) { return (ch < 4u) ? (int8_t)((ch % 2u) + 1) : (int8_t)1; }

/* Bitmask of the PX4 channels that belong to a group. */
static uint32_t group_mask(unsigned group)
{
	switch (group) {
	case 0: return 0x3;   /* ch0, ch1 */
	case 1: return 0xC;   /* ch2, ch3 */
	case 2: return 0x10;  /* ch4 */
	default: return 0;
	}
}

/* ub16 duty fraction for a pulse width (us) at a given rate (Hz). */
static ub16_t us_to_duty(uint16_t us, uint32_t rate)
{
	if (rate == 0) {
		return 0;
	}

	uint64_t duty = ((uint64_t)us * rate << 16) / 1000000ULL;

	if (duty > 0xffff) {
		duty = 0xffff;   /* clamp below 100% (ub16 max) */
	}

	return (ub16_t)duty;
}

/* Push the current rate + pulse widths of a group to hardware via start(). */
static void commit_group(unsigned group)
{
	struct pwm_lowerhalf_s *lower = g_lower[group];

	if (lower == NULL || !g_armed) {
		return;
	}

	struct pwm_info_s info;
	memset(&info, 0, sizeof(info));
	info.frequency = g_rate[group];

	if (group < 2) {
		/* EPWM module: two channels A/B */
		unsigned base_ch = group * 2u;
		info.channels[0].duty    = us_to_duty(g_pulse_us[base_ch],     g_rate[group]);
		info.channels[0].channel = 1;
		info.channels[1].duty    = us_to_duty(g_pulse_us[base_ch + 1], g_rate[group]);
		info.channels[1].channel = 2;

	} else {
		/* eCAP APWM: single channel */
		info.channels[0].duty    = us_to_duty(g_pulse_us[4], g_rate[group]);
		info.channels[0].channel = 1;
		info.channels[1].channel = -1;  /* terminate: no second channel */
	}

	lower->ops->start(lower, &info);
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	if (!g_inited) {
		int epwm_ok = 1;
		static int attempts;

		am67_epwm_init();

		/* J722S_DEV_EPWM0 / EPWM1. Setup reads PID 0 and skips pinmux
		 * while these power domains are off. */
		(void)am67_tisci_device_on(86u);
		(void)am67_tisci_device_on(87u);

		am67_ecap_init();

		g_lower[0] = am67_epwminitialize(0);
		g_lower[1] = am67_epwminitialize(1);
		g_lower[2] = am67_ecapinitialize(1);   /* eCAP1 = GPIO-16 (eCAP0 collides with EPWM0_B on C20) */

		for (unsigned g = 0; g < AM67_PWM_NGROUPS; g++) {
			int ret;

			if (g_lower[g] == NULL) {
				continue;
			}

			ret = g_lower[g]->ops->setup(g_lower[g]);

			if (g < 2u && ret < 0) {
				epwm_ok = 0;
			}
		}

		if (!epwm_ok && ++attempts < 3) {
			syslog(LOG_ERR, "epwm setup failed, retry %d\n", attempts);
			return -EIO;
		}

		g_epwm_ok = (epwm_ok != 0);
		g_inited = true;

		if (!g_epwm_ok) {
			syslog(LOG_ERR, "epwm setup failed, dropping channels 0-3\n");
		}
	}

	/* A dead EPWM must not stay in the mask: pwm_out would arm four
	 * silent motors and still look healthy. eCAP (channel 4) can run.
	 */
	if (!g_epwm_ok) {
		return (int)(channel_mask & ~0x0fu);
	}

	return (int)channel_mask;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set(unsigned channel, uint16_t value)
{
	if (channel >= AM67_PWM_NCHANNELS) {
		return -EINVAL;
	}

	g_pulse_us[channel] = value;
	commit_group(chan_group(channel));
	return OK;
}

uint16_t up_pwm_servo_get(unsigned channel)
{
	return (channel < AM67_PWM_NCHANNELS) ? g_pulse_us[channel] : 0;
}

void up_pwm_update(unsigned channels_mask)
{
	for (unsigned g = 0; g < AM67_PWM_NGROUPS; g++) {
		if (channels_mask & group_mask(g)) {
			commit_group(g);
		}
	}
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	if (group >= AM67_PWM_NGROUPS) {
		return -EINVAL;
	}

	g_rate[group] = rate;
	commit_group(group);
	return OK;
}

int up_pwm_servo_set_rate(unsigned rate)
{
	for (unsigned g = 0; g < AM67_PWM_NGROUPS; g++) {
		g_rate[g] = rate;
		commit_group(g);
	}

	return OK;
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return group_mask(group);
}

void up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	g_armed = armed;

	for (unsigned g = 0; g < AM67_PWM_NGROUPS; g++) {
		if (g_lower[g] == NULL) {
			continue;
		}

		if (armed) {
			commit_group(g);

		} else {
			g_lower[g]->ops->stop(g_lower[g]);
		}
	}
}
