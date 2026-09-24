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
 * @file init.c
 *
 * T3 Gemstone O1 (AM67 / J722S Cortex-R5F) PX4-specific board init.
 *
 * Called by NuttX during startup via boardctl(BOARDIOC_INIT) once
 * CONFIG_NSH_ARCHINIT is set. Kept minimal for the first bring-up: the SoC
 * clocks/muxes/power are already set by the Linux Device Manager before
 * remoteproc starts us, and the low-level pin/MPU/console setup happens in the
 * am67 arch layer (arch/arm/src/am67). All this needs to do is bring PX4's
 * platform (HRT, work queues, parameters, uORB) up.
 *
 * TODO (as arch-support lands): register SPI buses for the LPS22DF baro and
 * ICM-20948 IMU, start the sensor drivers, and configure any board GPIO.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/init.h>

#include <errno.h>
#include <syslog.h>
#include <nuttx/serial/uart_rpmsg_raw.h>

#include <nuttx/board.h>
#include <nuttx/arch.h>
#include <drivers/drv_hrt.h>

#include <parameters/param.h>

#include "board_config.h"

/* NuttX libc: runs the C++ static constructors (.init_array). It is guarded to
 * execute exactly once, so calling it here does not conflict with the call
 * NuttX makes later from nxtask_startup(). Not exposed in a public header, so
 * declare it locally. */
extern void lib_cxx_initialize(void);

/* arch/arm/src/am67/am67_serial.c: powers MAIN_UART6 (/dev/ttyS1, GPS). */
extern int am67_uart6_enable(void);

#ifdef CONFIG_RPTUN
/* AM67 rptun (RPMsg + virtio-net over the A53-Linux remoteproc link). Lives in
 * arch/arm/src/am67 (not on this board's include path), so declare it locally.
 * PX4 uses a custom board dir and does not compile the NuttX am67_bringup.c,
 * so the init call that file makes for the plain-NSH target must be made here. */
extern int am67_rptun_init(void);
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application-specific initialisation. Called by NuttX from
 *   boardctl(BOARDIOC_INIT).
 *
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;

	/* MAIN_UART6 is /dev/ttyS1 (GPS, HAT 7/11). The DMSC keeps it off and
	 * MDR1 disabled until asked; the GPS driver opens it from rcS, later. */
	if (am67_uart6_enable() < 0) {
		syslog(LOG_ERR, "[boot] MAIN_UART6 power-on failed, no GPS\n");
	}

	/* Run C++ global constructors before any PX4 code executes. This board
	 * brings PX4 up from board_late_initialize(), which NuttX calls from
	 * nx_start() BEFORE the init task's nxtask_startup() runs the static
	 * constructors. Without this, PX4 globals with non-zero initialisers -
	 * e.g. px4::atomic_bool _wq_manager_should_exit{true} - are still zero
	 * (they live in .bss), so the work-queue manager thinks it is already
	 * running and never starts, taking every driver down with it. The call is
	 * internally guarded to run the .init_array exactly once. */
	lib_cxx_initialize();

	/* Bring up the PX4 platform: this starts the HRT (DMTimer1), the high/low
	 * priority work queues, the parameter store and uORB. hrt_init() lives
	 * inside px4_platform_init(); if the HRT timer was not granted/clocked by
	 * the Device Manager it will complain loudly there (see the [hrt] logs). */
	int ret = px4_platform_init();

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] px4_platform_init failed (%d)\n", ret);
		return ret;
	}

#if defined(CONFIG_SCHED_CRITMONITOR) || defined(CONFIG_SCHED_IRQMONITOR)
	/* The critical-section and IRQ monitors time with the R5 PMU cycle
	 * counter. Nothing else starts it on this chip, so enable it here and
	 * calibrate it against the HRT (measured 799.93 MHz on the O1).
	 */
	{
		up_perf_init((void *)1);
		const hrt_abstime t0 = hrt_absolute_time();
		const clock_t c0 = up_perf_gettime();

		while (hrt_absolute_time() - t0 < 20000) {}

		const clock_t c1 = up_perf_gettime();
		const hrt_abstime t1 = hrt_absolute_time();
		const unsigned long hz = (unsigned long)((uint64_t)(c1 - c0) * 1000000ull / (t1 - t0));
		up_perf_init((void *)(uintptr_t)hz);
		syslog(LOG_INFO, "[perf] R5 core %lu Hz\n", hz);
	}
#endif

	/* Apply the board manifest (no-op until we describe HW variants). */
	px4_platform_configure();

	/* Map the PWM outputs to actuator functions as board defaults. This board
	 * has no persistent param storage and does not auto-run rcS/rc.board_defaults,
	 * so set the DEFAULT (not the value: no autosave, and a user override still
	 * wins) here to keep the outputs mapped across every boot:
	 *   ch0 EPWM0_A=Motor1  ch1 EPWM0_B=Motor2  ch2 EPWM1_A=Motor3
	 *   ch3 EPWM1_B=Motor4  ch4 eCAP1  =Motor5
	 */
	{
		static const char *const pwm_func_names[] = {
			"PWM_MAIN_FUNC1", "PWM_MAIN_FUNC2", "PWM_MAIN_FUNC3",
			"PWM_MAIN_FUNC4", "PWM_MAIN_FUNC5",
		};

		for (unsigned i = 0; i < sizeof(pwm_func_names) / sizeof(pwm_func_names[0]); i++) {
			param_t p = param_find(pwm_func_names[i]);

			if (p != PARAM_INVALID) {
				int32_t func = 101 + (int32_t)i;   /* Motor 1..5 */
				param_set_default_value(p, &func);
			}
		}
	}

#ifdef CONFIG_SPI
	/* Bring up MCU_MCSPI0 so the SPI sensor drivers (ICM-20948, ...) can bind.
	 * This powers the sensor rail and resets the MCSPI controller; watch the
	 * [spi] logs if it stalls (indicates the Device Manager did not clock the
	 * peripheral for this R5F). */
	am67_spidev_initialize();
#endif

#ifdef CONFIG_RPTUN
	/* Register the resource table (RPMsg + virtio-net) with NuttX's OpenAMP
	 * stack and enable the NAVSS mailbox IRQ. Linux is the remoteproc master
	 * and has already booted this R5F, so the vdev status/features are live in
	 * the resource table by the time we get here. Runs after px4_platform_init()
	 * so the work queues the rptun thread relies on are already up. */
	int rptun_ret = am67_rptun_init();

	if (rptun_ret < 0) {
		syslog(LOG_ERR, "[boot] am67_rptun_init failed (%d)\n", rptun_ret);
	}
#endif

	return OK;
}


#ifdef CONFIG_RPMSG_UART_RAW
/****************************************************************************
 * Name: rpmsg_serialrawinit
 *
 * Description:
 *   Called by drivers_initialize() when CONFIG_RPMSG_UART_RAW is set.
 *   The raw driver speaks unframed bytes on the fixed "rpmsg-tty"
 *   service, compatible with Linux's rpmsg_tty.  Empty devname registers
 *   as /dev/tty on NuttX; appears on Linux as /dev/ttyRPMSG0 once the
 *   name-service announcement lands.
 *
 ****************************************************************************/

void rpmsg_serialrawinit(void)
{
	uart_rpmsg_raw_init("r5f", "", 4096, false);
}
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Called by NuttX during OS bring-up (nx_start_application) when
 *   CONFIG_BOARD_LATE_INITIALIZE is set. On this board CONFIG_NSH_ARCHINIT is
 *   not enabled, so this is where PX4 gets brought up. Route it to the same
 *   board_app_initialize() used by the boardctl(BOARDIOC_INIT) path.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
	board_app_initialize(0);
}
#endif
