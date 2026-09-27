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
 * @file board_reset.c
 *
 * Board reset backend for the TI AM67 / J722S Cortex-R5F.
 *
 * Reset on this platform is fundamentally different from a self-contained
 * Cortex-M flight controller:
 *
 *   - The R5F is not the boot master. It is loaded and started by the Linux
 *     remoteproc framework running on the A-cores. The firmware image is
 *     chosen on the Linux side (/lib/firmware + remoteproc), not by us.
 *   - There is no Cortex-M style self-reset (no NVIC AIRCR/SYSRESETREQ), and
 *     NuttX does not implement up_systemreset() for this core.
 *   - A core reset belongs to Linux (`echo stop/start >
 *     /sys/class/remoteproc/.../state`), which also reloads the image; a
 *     running core cannot reset itself through the Device Manager.
 *
 * So the core asks Linux to restart it: board_reset() cuts the motor
 * outputs, sends RP_MBOX_CRASH on the remoteproc mailbox (Linux logs
 * "K3 R5F rproc ... crashed"; gem-r5f-restart.service answers with remoteproc
 * stop/start) and waits with interrupts on, so the rpmsg side can still
 * acknowledge the shutdown request.  A core that is locked (armed) refuses
 * that request and keeps running with the motors cut.
 *
 * From an interrupt handler or with interrupts already off (a crash path)
 * nothing else may run: the request is still sent, then the core halts
 * with interrupts off.  Linux cannot stop such a core; it needs a Linux
 * reboot.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <syslog.h>
#include <unistd.h>

/* arch/arm/src/am67/am67_rptun.c */
#ifdef CONFIG_RPTUN
extern void am67_rptun_request_restart(void);
#endif

#define rsterr(fmt, ...)  syslog(LOG_ERR, "[reset] " fmt "\n", ##__VA_ARGS__)

/* Motor output kill switches from arch/arm/src/am67 (Trip-Zone / eCAP stop).
 * Both are safe to call before the outputs were set up.
 */
#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)
extern void am67_epwm_emergency_stop(void);
#endif
#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || defined(CONFIG_AM67_ECAP2)
extern void am67_ecap_emergency_stop(void);
#endif

#ifdef CONFIG_BOARDCTL_RESET

/**
 * Configure a persistent reset "mode" (e.g. stay in bootloader on next boot).
 *
 * None of these modes are meaningful on a remoteproc-loaded R5F: the next
 * image is selected by Linux, and we own no persistent scratch register to
 * signal a bootloader. Report that honestly instead of pretending success.
 */
int board_configure_reset(reset_mode_e mode, uint32_t arg)
{
	(void)arg;

	switch (mode) {
	case BOARD_RESET_MODE_CLEAR:
		/* Nothing to clear - no persistent mode is ever stored. */
		return OK;

	default:
		rsterr("board_configure_reset(mode=%d) not supported: reset/boot mode "
		       "is controlled by Linux remoteproc, not the R5F.", (int)mode);
		return -ENOTSUP;
	}
}

#endif /* CONFIG_BOARDCTL_RESET */

/**
 * Reset the board.
 *
 * Called by NuttX boardctl(BOARDIOC_RESET) and, critically, by
 * board_crashdump() after a fatal fault. Never returns.
 */
int board_reset(int status)
{
	/* The EPWM and eCAP counters keep running without the CPU, so a halted
	 * core would leave every motor on its last command. Cut them first,
	 * before anything below can fail.
	 */
	irqstate_t flags = up_irq_save();
#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)
	am67_epwm_emergency_stop();
#endif
#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || defined(CONFIG_AM67_ECAP2)
	am67_ecap_emergency_stop();
#endif

	/* Interrupts were on at entry (bit 7 = I) and this is not a handler:
	 * the system is sane enough to wait for Linux.
	 */
	const bool can_wait = !up_interrupt_context() && (flags & (1u << 7)) == 0;

#ifdef CONFIG_RPTUN
	am67_rptun_request_restart();
#endif

	if (can_wait) {
		up_irq_restore(flags);
		rsterr("board_reset(status=%d): motors cut, asked Linux to restart this core", status);

		for (;;) {
			sleep(1);
		}
	}

	rsterr("board_reset(status=%d) from a handler or with interrupts off: halting; "
	       "recovering the core needs a Linux reboot.", status);

	for (;;) {
	}

	return 0; /* unreachable */
}
