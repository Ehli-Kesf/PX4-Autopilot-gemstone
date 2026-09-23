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
 * @file spi.cpp
 *
 * PX4 SPI arch-support glue for the TI AM67 / J722S (MCU_MCSPI0).
 *
 * Key difference from a typical PX4 board: the AM67 MCSPI drives its chip
 * selects in HARDWARE (per-channel FORCE bit), not via GPIO. There is no
 * cs_gpio to toggle. Instead each configured device carries its MCSPI channel
 * number in the px4_spi_bus_device_t::cs_gpio field (overloaded - see the
 * board's src/spi.cpp), and am67_spi0select() maps the incoming PX4 devid to
 * that channel and calls the NuttX driver's am67_mcspi_board_select().
 *
 * The NuttX am67 MCSPI driver's ops table references am67_spi0select /
 * am67_spi0status as external (board-provided) symbols; they are defined here.
 */

#include <board_config.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/spi.h>

#include <syslog.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>

__BEGIN_DECLS

/* NuttX am67 chip-driver entry points. Their prototypes live in the private
 * arch/arm/src/am67 headers, which are not on the PX4 include path, so the
 * minimal signatures are re-declared here (plain C symbols). */
void  am67_spiinitialize(void);                         /* controller + pinmux init */
struct spi_dev_s *am67_spibus_initialize(int port);
void  am67_mcspi_board_select(FAR struct spi_dev_s *dev, uint8_t channel, bool selected);
void  am67_sensors_power_enable(bool enable);

/* Board-provided select/status hooks consumed by the am67 MCSPI ops table. */
void    am67_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t am67_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);

__END_DECLS

/****************************************************************************
 * Private
 ****************************************************************************/

/* Look up the MCSPI hardware channel for a PX4 devid from the board's bus
 * table (cs_gpio holds the channel). Returns 0xff if the devid is unknown. */
static uint8_t am67_devid_to_channel(uint32_t devid)
{
	for (int b = 0; b < SPI_BUS_MAX_BUS_ITEMS; ++b) {
		if (px4_spi_buses[b].bus == -1) {
			break;
		}

		for (int d = 0; d < SPI_BUS_MAX_DEVICES; ++d) {
			const px4_spi_bus_device_t &dev = px4_spi_buses[b].devices[d];

			if (dev.cs_gpio != 0 && dev.devid == devid) {
				return (uint8_t)dev.cs_gpio;
			}
		}
	}

	return 0xff;
}

/****************************************************************************
 * NuttX driver hooks
 ****************************************************************************/

void am67_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	uint8_t channel = am67_devid_to_channel(devid);

	if (channel == 0xff) {
		syslog(LOG_ERR, "[spi] select: unknown devid 0x%08" PRIx32
		       " (not in px4_spi_buses)\n", devid);
		return;
	}

	am67_mcspi_board_select(dev, channel, selected);
}

uint8_t am67_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	(void)dev;
	return (am67_devid_to_channel(devid) != 0xff) ? SPI_STATUS_PRESENT : 0;
}

/****************************************************************************
 * PX4 board interface
 ****************************************************************************/

/*
 * Bring up the MCU_MCSPI0 controller. Called once from board_app_initialize().
 *
 * NOTE (suspect area): am67_spiinitialize() issues an MCSPI soft-reset and
 * busy-polls SYSSTATUS.RESETDONE. If the Linux Device Manager has not clocked
 * / de-asserted reset on MCU_MCSPI0 for this R5F, that poll never completes and
 * we hang here. The bracketing logs below make that unambiguous in the console.
 */
__EXPORT void am67_spidev_initialize(void)
{
	/* Power-cycle the onboard sensor rail (IMU_EN / MCU0 pin 12) so every
	 * chip on it gets a clean power-on reset. The rail switch floats between
	 * board power-up and the first driver of this pin, and a dirty VDD ramp
	 * can leave the LPS22DF latched unresponsive until a real POR; the
	 * ICM-20948 recovers via its driver's soft reset, the LPS22DF cannot. */
	am67_sensors_power_enable(false);
	usleep(100 * 1000);
	am67_sensors_power_enable(true);
	usleep(50 * 1000);
	/* Same drive again so the log shows the pin after the wait, not only at the write. */
	am67_sensors_power_enable(true);

	syslog(LOG_INFO, "[spi] MCU_MCSPI0 init: resetting controller...\n");
	am67_spiinitialize();
	syslog(LOG_INFO, "[spi] MCU_MCSPI0 init: done (CS driven in HW per channel)\n");
}

/****************************************************************************
 * PX4 board SPI power/reset helpers (referenced by the common layer)
 ****************************************************************************/

void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
	(void)bus_mask;
	am67_sensors_power_enable(enable_power);
}

void board_control_spi_sensors_power_configgpio()
{
	/* Sensor-rail enable is configured inside am67_sensors_power_enable(); no
	 * separate GPIO setup step is needed on this SoC. */
}

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	(void)bus_mask;

	/* Power-cycle the sensor rail: this is the only reset lever we have, since
	 * the SPI pads are muxed to the MCSPI function (not GPIO). */
	am67_sensors_power_enable(false);
	usleep(ms * 1000);
	am67_sensors_power_enable(true);
	usleep(100);
}
