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

using namespace lps22df;

/* Poll at the programmed ODR (25 Hz) */
static constexpr uint32_t LPS22DF_SAMPLE_INTERVAL = 1000000 / 25;

LPS22DF::LPS22DF(const I2CSPIDriverConfig &config, device::Device *interface) :
	I2CSPIDriver(config),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

LPS22DF::~LPS22DF()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int LPS22DF::init()
{
	if (configure() != OK) {
		return PX4_ERROR;
	}

	ScheduleNow();

	return PX4_OK;
}

int LPS22DF::configure()
{
	/* SPI only, so traffic to other devices on the bus is ignored; then
	 * block data update so multi-byte samples cannot tear, then start
	 * continuous conversion at 25 Hz.
	 */
	uint8_t if_ctrl = 0;
	int ret = _interface->read(IF_CTRL, &if_ctrl, 1);

	if (ret == OK) {
		ret = write_reg(IF_CTRL, if_ctrl | I2C_I3C_DIS);
	}

	if (ret == OK) {
		ret = write_reg(CTRL_REG2, BDU);
	}

	if (ret == OK) {
		ret = write_reg(CTRL_REG1, ODR_25HZ);
	}

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void LPS22DF::RunImpl()
{
	if (collect() != OK) {
		PX4_DEBUG("collection error");
		perf_count(_comms_errors);

		/* Reconfigure in case the device lost its settings */
		configure();
	}

	ScheduleDelayed(LPS22DF_SAMPLE_INTERVAL);
}

int LPS22DF::collect()
{
	perf_begin(_sample_perf);

	/* STATUS through TEMP_OUT_H in one auto-incrementing burst */
	struct {
		uint8_t	STATUS;
		uint8_t	PRESS_OUT_XL;
		uint8_t	PRESS_OUT_L;
		uint8_t	PRESS_OUT_H;
		uint8_t	TEMP_OUT_L;
		uint8_t	TEMP_OUT_H;
	} report{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(STATUS, (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_end(_sample_perf);
		return ret;
	}

	if ((report.STATUS & P_DA) == 0) {
		/* no new sample this cycle */
		perf_end(_sample_perf);
		return PX4_OK;
	}

	const uint32_t P = report.PRESS_OUT_XL + (report.PRESS_OUT_L << 8) + (report.PRESS_OUT_H << 16);
	const float pressure_pa = (P / PRESS_LSB_PER_HPA) * 100.f;

	const int16_t T = report.TEMP_OUT_L + (report.TEMP_OUT_H << 8);
	const float temperature = T / TEMP_LSB_PER_DEGC;

	_px4_baro.set_error_count(perf_event_count(_comms_errors));
	_px4_baro.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, pressure_pa);

	perf_end(_sample_perf);
	return PX4_OK;
}

int LPS22DF::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

void LPS22DF::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
