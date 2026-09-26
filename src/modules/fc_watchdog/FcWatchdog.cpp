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

#include "FcWatchdog.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <nuttx/irq.h>
#include <stdlib.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#if defined(CONFIG_AM67_RTI8_WDT)
extern "C" {
	int am67_rti_wdt_start(uint32_t timeout_ms);
	void am67_rti_wdt_kick(void);
	uint32_t am67_rti_wdt_counter(void);
	uint32_t am67_rti_wdt_status(void);
	void am67_rti_wdt_arm_fiq(void);
}
#else
# error "fc_watchdog needs a hardware watchdog backend (CONFIG_AM67_RTI8_WDT)"
#endif

ModuleBase::Descriptor FcWatchdog::desc{task_spawn, custom_command, print_usage};

FcWatchdog::FcWatchdog() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

FcWatchdog::~FcWatchdog()
{
	perf_free(_kick_perf);
	perf_free(_unhealthy_perf);
}

bool FcWatchdog::init(bool arm_fiq)
{
	_arm_fiq = arm_fiq;
	_timeout_ms = am67_rti_wdt_start(TIMEOUT_MS);

	if (_timeout_ms < 0) {
		PX4_ERR("watchdog start failed (%d)", _timeout_ms);
		return false;
	}

	// The preload has a 0.33 ms step; a larger difference means the watchdog
	// was already running.
	if (abs(_timeout_ms - (int)TIMEOUT_MS) > 5) {
		// Already running since a remoteproc restart: its preload cannot change.
		PX4_WARN("watchdog timeout %d ms (set by an earlier image)", _timeout_ms);
	}

	ScheduleOnInterval(KICK_INTERVAL);
	return true;
}

void FcWatchdog::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_angular_velocity_sub.update(&angular_velocity)) {
		_last_gyro = angular_velocity.timestamp;
	}

	actuator_motors_s actuator_motors;

	if (_actuator_motors_sub.update(&actuator_motors)) {
		_last_motors = actuator_motors.timestamp;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_gyro < FRESH_MAX && now - _last_motors < FRESH_MAX) {
		am67_rti_wdt_kick();
		perf_count(_kick_perf);

		// Route the expiry to the motor cut only once the loop has been
		// healthy for a while, so a slow boot never trips it.
		if (_arm_fiq && !_fiq_armed && ++_healthy_kicks >= KICKS_BEFORE_ARM) {
			am67_rti_wdt_arm_fiq();
			_fiq_armed = true;
			PX4_INFO("armed: expiry cuts the motor outputs");
		}

	} else {
		perf_count(_unhealthy_perf);
	}
}

int FcWatchdog::print_status()
{
	PX4_INFO("timeout %d ms, motor cut %s", _timeout_ms,
		 _fiq_armed ? "armed" : (_arm_fiq ? "pending" : "disabled (-n)"));
	PX4_INFO("counter 0x%08" PRIx32 ", status 0x%02" PRIx32,
		 am67_rti_wdt_counter(), am67_rti_wdt_status());
	perf_print_counter(_kick_perf);
	perf_print_counter(_unhealthy_perf);
	return 0;
}

int FcWatchdog::task_spawn(int argc, char *argv[])
{
	bool arm_fiq = true;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "n", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'n':
			arm_fiq = false;
			break;

		default:
			return print_usage("unknown option");
		}
	}

	FcWatchdog *instance = new FcWatchdog();

	if (instance == nullptr) {
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init(arm_fiq)) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int FcWatchdog::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[0], "test") && !strcmp(argv[1], "hang")) {
		// Bench test only: hang this task with interrupts off. The watchdog
		// must cut the motors; recover with remoteproc stop/start.
		if (!is_running(desc) || !get_instance<FcWatchdog>(desc)->_fiq_armed) {
			PX4_ERR("watchdog not armed");
			return PX4_ERROR;
		}

		uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
		actuator_armed_s armed{};

		if (!armed_sub.copy(&armed) || armed.armed) {
			PX4_ERR("refused: vehicle armed");
			return PX4_ERROR;
		}

		PX4_WARN("hanging with interrupts off");
		px4_usleep(100_ms);
		(void)up_irq_save();

		for (;;) {}
	}

	return print_usage("unknown command");
}

int FcWatchdog::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Feeds the hardware watchdog only while the gyro and the motor outputs are
being published. If the flight control loop stops, or the core hangs, the
watchdog expires and cuts every motor output.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fc_watchdog", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('n', "Kick only, do not route the expiry to the motor cut", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "test hang: hang with interrupts off (bench only)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fc_watchdog_main(int argc, char *argv[])
{
	return ModuleBase::main(FcWatchdog::desc, argc, argv);
}
