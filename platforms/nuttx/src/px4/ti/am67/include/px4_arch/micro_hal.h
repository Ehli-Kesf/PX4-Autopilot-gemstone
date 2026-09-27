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
 * @file px4_arch/micro_hal.h
 *
 * PX4 micro-HAL for the TI AM67 / J722S Cortex-R5F. Bridges the generic PX4
 * arch abstraction to the NuttX am67 chip drivers.
 */

#pragma once

#include <px4_platform/micro_hal.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* px4_platform/micro_hal.h maps px4_enter_critical_section() to NuttX's
 * enter_critical_section(); pull in its declaration (the STM/NXP arch headers
 * get this transitively via their chip headers, we must do it explicitly). */
#include <nuttx/irq.h>

/* Full definition of struct i2c_master_s and struct i2c_msg_s. PX4 I2C users
 * (e.g. the i2cdetect systemcmd, which builds i2c_msg_s message vectors) need
 * the complete type, not just a forward declaration. */
#include <nuttx/i2c/i2c_master.h>

__BEGIN_DECLS

/* Forward declarations of the NuttX am67 chip-driver entry points we bridge
 * to. Their full prototypes live in the private arch/arm/src/am67 headers,
 * which are not on the PX4 include path, so re-declare the
 * minimal signatures here (C linkage - they are plain NuttX C symbols). */
struct spi_dev_s;
struct spi_dev_s   *am67_spibus_initialize(int port);
struct i2c_master_s *am67_i2cbus_initialize(int port);
int                 am67_i2cbus_uninitialize(struct i2c_master_s *dev);

/* am67 GPIO primitives. am67_gpio_t is a plain uint32_t "pin id" (an index
 * into the SoC's board pad table, see arch/arm/src/am67/am67_gpio.h), and
 * pintype is GPIO_OUTPUT(0)/GPIO_INPUT(1). */
void                am67_configgpio(uint32_t gpio, int pintype);
void                am67_gpiowrite(uint32_t gpio, bool value);
bool                am67_gpioread(uint32_t gpio);

/* SoC architecture id embedded in the PX4 GUID. Not in the core enum (which
 * only lists PX4-supported chips), so use a distinct literal for the am67. */
#define PX4_SOC_ARCH_ID                         0x00a6u

/* ---- CPU UUID -----------------------------------------------------------
 * TODO: read a real unique ID from the SoC (e.g. an eFuse/OTP field). For now
 * only the length/format macros are provided so the version code compiles;
 * board_get_uuid()/mfguid() come from the common layer. */
#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH / sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH
#define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH - 1 + (2 * PX4_CPU_UUID_BYTE_LENGTH) + 1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2 * PX4_CPU_MFGUID_BYTE_LENGTH) + 1)

/* No battery-backed SRAM crash store on this remoteproc-loaded core, so no
 * px4_savepanic(): defining it makes the logger expect HARDFAULT_ULOG_PATH.
 */

/* ---- Buses --------------------------------------------------------------
 * am67 SPI/I2C ports are 0-based (NuttX). */
/* PX4 addresses SPI/I2C buses 1-based; the am67 NuttX driver ports are 0-based
 * (MCU_MCSPI0 is port 0, MCU_I2C0 is port 0). Map here. */
#define PX4_BUS_OFFSET                          1
#define px4_spibus_initialize(bus_num)          am67_spibus_initialize((bus_num) - PX4_BUS_OFFSET)
#define px4_i2cbus_initialize(bus_num)          am67_i2cbus_initialize((bus_num) - PX4_BUS_OFFSET)
#define px4_i2cbus_uninitialize(pdev)           am67_i2cbus_uninitialize(pdev)

/* ---- GPIO ---------------------------------------------------------------
 * PX4 uses a single packed uint32_t 'pinset'. The am67 API is instead a plain
 * pin-id + direction, so we pack them: bits[7:0]=am67 pin id, bit8=direction
 * (1=input), bit9=initial output level, bit31=validity marker. The marker lets
 * us safely ignore any foreign (e.g. STM32-style) pinset a generic PX4 caller
 * might hand us - only pins built with the AM67_GPIO_* macros below are acted
 * on. Note: the on-chip SPI chip-selects are hardware-driven (MCSPI FORCE bit,
 * see the arch spi layer), NOT via these GPIO ops. */
#define _AM67_GPIO_MARK                         (1u << 31)  /* "this is an am67 pinset" */
#define _AM67_GPIO_IN                           (1u << 8)   /* set: input, clear: output */
#define _AM67_GPIO_HIGH                         (1u << 9)   /* initial level for outputs */
#define _AM67_GPIO_ID_MASK                      0xffu

#define AM67_GPIO_OUTPUT(id)                    (_AM67_GPIO_MARK | ((id) & _AM67_GPIO_ID_MASK))
#define AM67_GPIO_OUTPUT_HIGH(id)               (_AM67_GPIO_MARK | _AM67_GPIO_HIGH | ((id) & _AM67_GPIO_ID_MASK))
#define AM67_GPIO_INPUT(id)                     (_AM67_GPIO_MARK | _AM67_GPIO_IN | ((id) & _AM67_GPIO_ID_MASK))

/* Referenced by the generic SPIBusIterator (platforms/common/spi.cpp) to
 * extract a chip-select index from a device's cs_gpio field. On the am67,
 * cs_gpio holds the MCSPI channel number directly (see the board's spi.cpp),
 * so the mask is just the low byte (channels 0..3). */
#ifndef GPIO_PIN_MASK
#  define GPIO_PIN_MASK                         0xffu
#endif

static inline int px4_arch_configgpio(uint32_t pinset)
{
	if ((pinset & _AM67_GPIO_MARK) == 0) {
		return 0;
	}

	uint32_t id = pinset & _AM67_GPIO_ID_MASK;

	if ((pinset & _AM67_GPIO_IN) != 0) {
		am67_configgpio(id, 1 /* GPIO_INPUT */);

	} else {
		am67_configgpio(id, 0 /* GPIO_OUTPUT */);
		am67_gpiowrite(id, (pinset & _AM67_GPIO_HIGH) != 0);
	}

	return 0;
}

static inline int px4_arch_unconfiggpio(uint32_t pinset)
{
	if ((pinset & _AM67_GPIO_MARK) != 0) {
		am67_configgpio(pinset & _AM67_GPIO_ID_MASK, 1 /* return to input */);
	}

	return 0;
}

static inline bool px4_arch_gpioread(uint32_t pinset)
{
	if ((pinset & _AM67_GPIO_MARK) == 0) {
		return false;
	}

	return am67_gpioread(pinset & _AM67_GPIO_ID_MASK);
}

static inline int px4_arch_gpiowrite(uint32_t pinset, bool value)
{
	if ((pinset & _AM67_GPIO_MARK) != 0) {
		am67_gpiowrite(pinset & _AM67_GPIO_ID_MASK, value);
	}

	return 0;
}

static inline int px4_arch_gpiosetevent(uint32_t pinset, bool r, bool f, bool e,
					xcpt_t func, void *arg)
{
	/* No GPIO interrupt/event layer on the am67 yet: sensors that would use a
	 * data-ready IRQ must run in polled mode (drdy_gpio == 0). Returning OK
	 * (not an error) because a poll-mode driver never registers an event. */
	(void)pinset; (void)r; (void)f; (void)e; (void)func; (void)arg;
	return 0;
}

#define PX4_MAKE_GPIO_INPUT(gpio)                (((gpio) & ~_AM67_GPIO_HIGH) | _AM67_GPIO_IN)
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio)         ((gpio) & ~(_AM67_GPIO_IN | _AM67_GPIO_HIGH))
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio)           (((gpio) & ~_AM67_GPIO_IN) | _AM67_GPIO_HIGH)
#define PX4_GPIO_PIN_OFF(gpio)                   (((gpio) & ~_AM67_GPIO_HIGH) | _AM67_GPIO_IN)

/* ---- HRT ----------------------------------------------------------------
 * DMTimer1 free-running counter rate (see board_config.h / ti/am67/hrt). */
#ifndef HRT_TIMER_RATE
#  define HRT_TIMER_RATE                         25000000u
#endif
#define TIMER_HRT_CYCLES_PER_US                 (HRT_TIMER_RATE / 1000000)
#define TIMER_HRT_CYCLES_PER_MS                 (HRT_TIMER_RATE / 1000)

/* ---- Cache --------------------------------------------------------------
 * No data-cache management wired for this configuration. */
#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc                 malloc

__END_DECLS
