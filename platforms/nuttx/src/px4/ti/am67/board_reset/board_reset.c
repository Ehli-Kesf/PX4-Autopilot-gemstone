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
 *   - A genuine core reset is owned by the K3 Device Manager (TISCI) or by
 *     Linux (`echo stop/start > /sys/class/remoteproc/.../state`). We have no
 *     TISCI transport here yet.
 *
 * So we cannot truthfully "reboot" ourselves. Rather than silently doing
 * nothing (which would let a corrupted OS keep running after an assert - see
 * board_crashdump.c, which calls board_reset() precisely because RAM is
 * already trashed), board_reset() logs loudly and parks the core in a defined
 * halted state with interrupts disabled. If remoteproc crash-recovery is
 * enabled on the Linux side, it can then detect the stalled core and reload
 * the firmware.
 *
 * TODO: once a TISCI transport exists, replace the halt with a proper core
 * reset request (or an mbox/IPC ping asking Linux to restart us).
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/shutdown.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <syslog.h>

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
	up_irq_save();
#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)
	am67_epwm_emergency_stop();
#endif
#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || defined(CONFIG_AM67_ECAP2)
	am67_ecap_emergency_stop();
#endif

	if (status == REBOOT_TO_BOOTLOADER) {
		rsterr("reboot-to-bootloader requested but unsupported on this core "
		       "(remoteproc selects the image).");

	} else {
		rsterr("board_reset(status=%d) requested.", status);
	}

	rsterr("No self-reset path on the remoteproc-loaded R5F "
	       "(no up_systemreset()/TISCI). Halting core with IRQs disabled.");
	rsterr("Recover from the Linux host, e.g.:");
	rsterr("  echo stop  > /sys/class/remoteproc/remoteprocN/state");
	rsterr("  echo start > /sys/class/remoteproc/remoteprocN/state");

	/* Freeze in a defined state so a corrupted OS cannot keep running and so
	 * remoteproc crash-recovery (if enabled) can take over. Interrupts have
	 * been off since entry. */
	for (;;) {
	}

	return 0; /* unreachable */
}
