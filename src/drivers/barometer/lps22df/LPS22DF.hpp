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

/**
 * @file LPS22DF.hpp
 *
 * Driver for the ST LPS22DF barometer connected via SPI.
 *
 * The device runs in continuous mode: the ODR is programmed once and the
 * driver polls the STATUS/output registers at the same rate.  Register
 * addresses, field encodings and sensitivities follow the LPS22DF
 * datasheet (also mirrored by the Linux st_pressure driver).
 */

#pragma once

#include <drivers/device/Device.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

namespace lps22df
{

/* IF_CTRL: I2C_I3C_DIS in bit 6. With I2C/I3C enabled (reset default) the
 * part also listens while CS is high and can take SPI traffic to other
 * devices on the bus for I2C/I3C. ST's driver sets it for SPI use.
 */
static constexpr uint8_t IF_CTRL = 0x0E;
static constexpr uint8_t I2C_I3C_DIS = 1 << 6;

static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t WHO_AM_I_VALUE = 0xB4;

/* CTRL_REG1: ODR in bits 6:3 (0 = power-down / one-shot mode) */
static constexpr uint8_t CTRL_REG1 = 0x10;
static constexpr uint8_t ODR_25HZ = 0x04 << 3;

/* CTRL_REG2: BDU in bit 3 (block data update: output regs are not
 * updated between reads of a sample's LSB and MSB)
 */
static constexpr uint8_t CTRL_REG2 = 0x11;
static constexpr uint8_t BDU = 1 << 3;

/* STATUS: P_DA bit 0 (pressure data available), T_DA bit 1 */
static constexpr uint8_t STATUS = 0x27;
static constexpr uint8_t P_DA = 1 << 0;

/* Output registers, address auto-increment (enabled by default) */
static constexpr uint8_t PRESS_OUT_XL = 0x28;

/* Sensitivities: 4096 LSB/hPa, 100 LSB/degC */
static constexpr float PRESS_LSB_PER_HPA = 4096.f;
static constexpr float TEMP_LSB_PER_DEGC = 100.f;

} // namespace lps22df

/* interface factory */
extern device::Device *LPS22DF_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class LPS22DF : public I2CSPIDriver<LPS22DF>
{
public:
	LPS22DF(const I2CSPIDriverConfig &config, device::Device *interface);
	virtual ~LPS22DF();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int	init();
	void	print_status();
	void	RunImpl();

private:
	int	configure();
	int	collect();

	int	write_reg(uint8_t reg, uint8_t val);

	PX4Barometer		_px4_baro;
	device::Device		*_interface;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
};
