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
 * @file board_config.h
 *
 * PX4 board-configuration for the T3 Gemstone O1 (TI AM67 / J722S Cortex-R5F,
 * loaded by Linux remoteproc).
 *
 * This is intentionally minimal for the first "boot + SPI sensors" milestone.
 * The AM67 has no PX4 ADC / PWM-output (io_timer) / USB layer yet, so battery
 * monitoring, RC input and actuator outputs are stubbed out below and marked
 * TODO. Fill them in as the corresponding arch-support pieces land.
 */

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* PX4 High-Resolution Timer -------------------------------------------------
 *
 * These override the defaults baked into the arch HRT
 * (platforms/nuttx/src/px4/ti/am67/hrt/hrt.c). DMTimer0 @ 0x02400000 is the
 * NuttX system tick; the HRT uses DMTimer1. Change here (not in arch code) if
 * a different spare DMTimer must be used.
 */
#define HRT_TIMER_BASE   0x02410000u   /* MAIN DMTimer1                          */
#define HRT_TIMER_IRQ    25            /* R5FSS0_CORE0 TIMER1 pending interrupt   */
#define HRT_TIMER_RATE   25000000u     /* DMTimer input = HFOSC0 @ 25 MHz         */

/* I2C ----------------------------------------------------------------------
 * The only I2C bus enabled and wired on this board is WKUP_I2C0 (WKUP domain,
 * 0x2b200000) -> PX4 bus 3. The AM67 also has MCU_I2C0 (MCU domain,
 * 0x04900000 -> PX4 bus 1), but it carries no peripheral and is not built, so
 * it is not listed. See src/i2c.cpp for the table and mapping.
 */
#define PX4_NUMBER_I2C_BUSES   1

/* Skip px4_platform_i2c_init()'s eager boot-time probe (a general-call software
 * reset transfer on every declared bus). It is unsafe here: (a) it does not
 * null-check the bus handle, so a bus whose NuttX port is not enabled crashes
 * on I2C_TRANSFER(NULL); and (b) the AM67 I2C functional clock is enabled by
 * the Device Manager only later (the am67_i2c driver brings HW up lazily on the
 * first real transfer), so touching the bus this early is invalid. I2C is
 * instead initialised on demand by i2cdetect / sensor drivers. */
#define BOARD_I2C_LATEINIT 1

/* SPI ----------------------------------------------------------------------
 * MCU_MCSPI0 = PX4 logical bus 1 (maps to am67 NuttX port 0, see
 * PX4_BUS_OFFSET in px4_arch/micro_hal.h). Chip selects are driven in HARDWARE
 * by the MCSPI (per-channel FORCE bit), so there are NO chip-select GPIOs; the
 * board's src/spi.cpp encodes each device's MCSPI channel in the
 * px4_spi_bus_device_t::cs_gpio field and the arch spi layer maps devid ->
 * channel from px4_spi_buses.
 */
#define PX4_SPI_BUS_SENSORS    1

/* MCSPI channel numbers per the Gemstone O1 wiring / Linux DTS. */
#define AM67_MCSPI_CH_LPS22DF  1     /* MCU_SPI0 CS1 */
#define AM67_MCSPI_CH_ICM20948 3     /* MCU_SPI0 CS3 */

/* Power / battery ----------------------------------------------------------
 * No ADC layer on the AM67 yet -> no power bricks. board_common.h explicitly
 * allows BOARD_NUMBER_BRICKS == 0.
 */
#define BOARD_NUMBER_BRICKS    0

/* PWM outputs --------------------------------------------------------------
 * Actuator outputs via the PX4 pwm_out module. The am67 arch pwm_servo layer
 * (platforms/nuttx/src/px4/ti/am67/io_pins) drives the copper-verified NuttX
 * EPWM/eCAP lower-halves. Channel/group map (see src/timer_config.cpp):
 *   group 0 = EPWM0  -> ch 0 (A), ch 1 (B)
 *   group 1 = EPWM1  -> ch 2 (A), ch 3 (B)
 *   group 2 = eCAP1  -> ch 4 (APWM, HAT GPIO-16 / A25)
 * EPWM2 is the cooling fan and is deliberately excluded. eCAP0 is unusable: its
 * only HAT pad (C20) collides with EPWM0_B, so ch4 uses eCAP1 instead.
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   5
#define BOARD_NUM_IO_TIMERS          3

/* RC input -----------------------------------------------------------------
 * board_common.h requires RC_SERIAL_PORT or CONFIG_BOARD_SERIAL_RC. No RC
 * receiver is wired and rcS does not start an RC driver. /dev/ttyS1 is the
 * GPS UART, so the placeholder names a node that does not exist; opening it
 * fails instead of stealing the GPS port. Point it at the RC UART in D5.
 */
#define RC_SERIAL_PORT         "/dev/ttyS9"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__BEGIN_DECLS

/* Bring up the MCU_MCSPI0 controller (arch spi layer,
 * platforms/nuttx/src/px4/ti/am67/spi). Called from board_app_initialize(). */
void am67_spidev_initialize(void);

__END_DECLS

#include <px4_platform_common/board_common.h>
