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
 * So the core resets the whole SoC itself: board_reset() cuts the motor
 * outputs and asks the Device Manager for a SoC reset (TISCI SYS_RESET, the
 * request Linux reboot sends). U-Boot then starts this core again from the
 * SD boot partition (early boot); Linux restarts too and attaches. Nothing
 * waits for Linux: a reset needed in a hurry happens at once. The log
 * (RAMLOG) lives in DDR that the reset does not clear, so the next start
 * still shows why the previous one ended.
 *
 * From an interrupt handler or with interrupts off (assert, crash) the
 * same happens with the lock-free request. Only if the DM refuses does the
 * core fall back to asking Linux (remoteproc mailbox) and halting.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>

#include <nuttx/board.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <syslog.h>
#include <unistd.h>

/* arch/arm/src/am67/am67_rptun.c */
#ifdef CONFIG_RPTUN
extern void am67_rptun_request_restart(void);
#endif

#define rsterr(fmt, ...)  syslog(LOG_ERR, "[reset] " fmt "\n", ##__VA_ARGS__)

/* arch/arm/src/am67/am67_tisci.c */
extern int am67_tisci_sys_reset(void);
extern int am67_tisci_sys_reset_now(void);

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

	/* Interrupts were on at entry (bit 7 = I) and this is not a handler */
	const bool thread = !up_interrupt_context() && (flags & (1u << 7)) == 0;

	rsterr("board_reset(status=%d): motors cut, resetting the SoC", status);

	int ret;

	if (thread) {
		/* The Trip-Zone holds the outputs low on its own; the locked
		 * request may wait for another TISCI user, so not with IRQs off.
		 */
		up_irq_restore(flags);
		up_flush_dcache_all(); /* the log must reach DDR */
		ret = am67_tisci_sys_reset();

	} else {
		up_flush_dcache_all();
		ret = am67_tisci_sys_reset_now();
	}

	/* The DM refused: the last resort is Linux */
	rsterr("SoC reset refused (%d): asking Linux to restart this core", ret);
#ifdef CONFIG_RPTUN
	am67_rptun_request_restart();
#endif

	if (thread) {
		for (;;) {
			sleep(1);
		}
	}

	for (;;) {
	}

	return 0; /* unreachable */
}
