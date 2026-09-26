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
 * @file FcWatchdog.hpp
 *
 * Feeds the hardware watchdog only while the flight control loop is alive:
 * the gyro (vehicle_angular_velocity) and the motor outputs
 * (actuator_motors) must both have been published within FRESH_MAX. If
 * either stops for longer than the watchdog timeout, or the core hangs,
 * the watchdog fires and cuts the motor outputs.
 *
 * The timeout is longer than a sensor driver reset (about 250 ms for the
 * ICM-20948), so a transient reset does not cut the motors, while a hung
 * core or a dead sensor path does within half a second.
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_angular_velocity.h>

using namespace time_literals;

class FcWatchdog : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	FcWatchdog();
	~FcWatchdog() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init(bool arm_fiq);
	int print_status() override;

private:
	void Run() override;

	static constexpr uint32_t TIMEOUT_MS = 500;
	static constexpr hrt_abstime FRESH_MAX = 50_ms;
	static constexpr hrt_abstime KICK_INTERVAL = 10_ms;
	static constexpr unsigned KICKS_BEFORE_ARM = 100;  // 1 s of healthy loop

	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	hrt_abstime _last_gyro{0};
	hrt_abstime _last_motors{0};

	bool _arm_fiq{true};
	bool _fiq_armed{false};
	unsigned _healthy_kicks{0};
	int _timeout_ms{0};

	perf_counter_t _kick_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": kick interval")};
	perf_counter_t _unhealthy_perf{perf_alloc(PC_COUNT, MODULE_NAME": unhealthy cycles")};
};
