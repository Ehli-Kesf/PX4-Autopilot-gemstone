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

#include "LPS22DF.hpp"

#include <drivers/device/spi.h>
#include <px4_platform_common/log.h>

#include <inttypes.h>

/* SPI protocol address bits */
#define DIR_READ	(1 << 7)
#define DIR_WRITE	(0 << 7)

device::Device *LPS22DF_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class LPS22DF_SPI : public device::SPI
{
public:
	LPS22DF_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	~LPS22DF_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;
};

device::Device *
LPS22DF_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode)
{
	return new LPS22DF_SPI(bus, devid, bus_frequency, spi_mode);
}

LPS22DF_SPI::LPS22DF_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_LPS22DF, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int LPS22DF_SPI::probe()
{
	/* Datasheet map: IF_CTRL 0x0E, WHO_AM_I 0x0F, CTRL_REG1 0x10, CTRL_REG2 0x11, STATUS 0x27. */
	static const uint8_t probe_regs[] = {
		0x0e, lps22df::WHO_AM_I, lps22df::CTRL_REG1, lps22df::CTRL_REG2, lps22df::STATUS
	};

	for (uint8_t reg : probe_regs) {
		uint8_t unused = 0;
		(void)read(reg, &unused, 1);
	}

	/* Second pass: are 0x11 and 0x27 stable. 32-bit RX is logged in the MCSPI driver. */
	{
		uint8_t repeat = 0;
		(void)read(lps22df::CTRL_REG2, &repeat, 1);
		(void)read(lps22df::STATUS, &repeat, 1);
	}

	uint8_t id = 0;
	const int ret = read(lps22df::WHO_AM_I, &id, 1);

	if (ret != 0) {
		PX4_ERR("WHO_AM_I read failed (%i)", ret);
		return -EIO;
	}

	if (id != lps22df::WHO_AM_I_VALUE) {
		PX4_ERR("WHO_AM_I 0x%02x, expected 0x%02x", id, lps22df::WHO_AM_I_VALUE);
		return -EIO;
	}

	return PX4_OK;
}

int LPS22DF_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	const uint8_t cmd = (uint8_t)(address | DIR_READ);
	buf[0] = cmd;

	int ret = transfer(&buf[0], &buf[0], count + 1);

	/* Same buffer is sent and received, so buf[0] is no longer the command.
	 * Probe window only, once per address: IF_CTRL, WHO_AM_I, CTRL_REG1/2, STATUS.
	 */
	if (count >= 1 && (address == 0x0e || address == lps22df::WHO_AM_I ||
			   address == lps22df::CTRL_REG1 || address == lps22df::CTRL_REG2 ||
			   address == lps22df::STATUS)) {
		static uint32_t logged_mask;

		if ((logged_mask & (1u << (address & 31))) == 0) {
			logged_mask |= 1u << (address & 31);
			PX4_ERR("reg 0x%02x cmd 0x%02x rx0 0x%02x rx1 0x%02x ret %i bus %u freq %" PRIu32,
				address, cmd, buf[0], buf[1], ret,
				get_device_bus(), get_frequency());
		}
	}

	memcpy(data, &buf[1], count);
	return ret;
}

int LPS22DF_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}
