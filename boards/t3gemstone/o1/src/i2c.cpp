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
 * @file i2c.cpp
 *
 * Board-specific I2C bus configuration (the px4_i2c_buses table used by
 * platforms/common/i2c.cpp and PX4 I2C sensor drivers).
 *
 * PX4 bus N maps to am67 NuttX port N - PX4_BUS_OFFSET (px4_arch/micro_hal.h):
 * PX4 bus 1 is MCU_I2C0 (0x04900000), on HAT 3 (SDA) and HAT 5 (SCL). It is
 * the external bus for a GPS-mounted compass. The px4-r5f overlay disables it
 * in Linux, and am67_i2c powers it through the DMSC on first use.
 *
 * WKUP_I2C0 is not listed and not built: Linux drives the PMIC, RTC and
 * EEPROM on it. The am67 I2C driver resets its controller on first use, so a
 * single R5F transfer there would cut into Linux's PMIC traffic.
 */

#include <px4_platform_common/i2c.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	{ 1, true },    /* MCU_I2C0 (am67 port 0), HAT 3/5, external */
};
