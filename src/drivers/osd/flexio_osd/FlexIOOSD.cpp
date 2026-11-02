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

#include "FlexIOOSD.hpp"

#include <lib/modes/ui.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include <errno.h>
#include <stdio.h>

ModuleBase::Descriptor FlexIOOSD::desc{task_spawn, custom_command, print_usage};

// Analog monitors crop the edges of the picture, keep the text inside the title-safe area
static constexpr int MARGIN_X = 4;
static constexpr int MARGIN_Y = 3;

static constexpr int CROSSHAIR_HALF_LENGTH = 4;

static constexpr float NS_PER_US = 1000.f;

FlexIOOSD::FlexIOOSD() :
	ModuleParams(nullptr)
{
}

int FlexIOOSD::configure_overlay()
{
	switch (_param_osd_flx_std.get()) {
	case STANDARD_PAL:
		_standard = FLEXIO_OSD_STANDARD_PAL;
		break;

	case STANDARD_NTSC:
		_standard = FLEXIO_OSD_STANDARD_NTSC;
		break;

	default: {
			// Until video is detected keep the timing it had, PAL at startup
			int detected = up_flexio_osd_detected_standard();

			if (detected >= 0) {
				_standard = static_cast<flexio_osd_standard_t>(detected);
			}
		}
		break;
	}

	const int32_t h_offset_ns = static_cast<int32_t>(_param_osd_flx_h_ofs.get() * NS_PER_US);
	// Rejected settings leave the overlay running as it was
	int ret = up_flexio_osd_init(_standard, h_offset_ns, _param_osd_flx_v_ofs.get());

	if (ret == -ENODEV) {
		PX4_ERR("board has no FlexIO OSD");

	} else if (ret != PX4_OK) {
		PX4_ERR("overlay init failed (%i)", ret);
	}

	return ret;
}

void FlexIOOSD::follow_detected_standard()
{
	if (_param_osd_flx_std.get() != STANDARD_AUTO) {
		return;
	}

	int detected = up_flexio_osd_detected_standard();

	if (detected >= 0 && detected != _standard) {
		PX4_INFO("%s video detected", detected == FLEXIO_OSD_STANDARD_PAL ? "PAL" : "NTSC");
		configure_overlay();
	}
}

void FlexIOOSD::update_topics()
{
	battery_status_s battery;

	if (_battery_status_sub.update(&battery)) {
		_battery_valid = battery.connected;
		_battery_voltage_v = battery.voltage_v;
		_battery_remaining = battery.remaining;
	}

	vehicle_local_position_s local_position;

	if (_vehicle_local_position_sub.update(&local_position)) {
		_altitude_valid = local_position.z_valid;
		_altitude_m = -local_position.z;
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		_armed_time = vehicle_status.armed_time;
		_nav_state = vehicle_status.nav_state_display;
		_vehicle_type = vehicle_status.vehicle_type;
	}
}

void FlexIOOSD::render()
{
	char text[24];

	_canvas.clear();

	const int top = MARGIN_Y;
	const int left = MARGIN_X;
	const int right = Canvas::WIDTH - MARGIN_X;
	const int center = Canvas::WIDTH / 2;
	const int bottom_line = Canvas::HEIGHT - MARGIN_Y - Canvas::CHAR_HEIGHT;
	const int status_line = bottom_line - Canvas::CHAR_HEIGHT;

	if (_battery_valid) {
		snprintf(text, sizeof(text), "%.1fV", (double)_battery_voltage_v);
		_canvas.draw_text(left, top, text);

		if (_battery_remaining >= 0.f) {
			snprintf(text, sizeof(text), "%d%%", static_cast<int>(_battery_remaining * 100.f + 0.5f));
			_canvas.draw_text(right, top, text, Canvas::Align::Right);
		}
	}

	// Crosshair, white with a black outline
	const int center_y = Canvas::HEIGHT / 2;

	for (int offset = -1; offset <= 1; offset += 2) {
		_canvas.draw_hline(center - CROSSHAIR_HALF_LENGTH - 1, center_y + offset, 2 * CROSSHAIR_HALF_LENGTH + 3,
				   FLEXIO_OSD_BLACK);
		_canvas.draw_vline(center + offset, center_y - CROSSHAIR_HALF_LENGTH - 1, 2 * CROSSHAIR_HALF_LENGTH + 3,
				   FLEXIO_OSD_BLACK);
	}

	_canvas.draw_hline(center - CROSSHAIR_HALF_LENGTH, center_y, 2 * CROSSHAIR_HALF_LENGTH + 1, FLEXIO_OSD_WHITE);
	_canvas.draw_vline(center, center_y - CROSSHAIR_HALF_LENGTH, 2 * CROSSHAIR_HALF_LENGTH + 1, FLEXIO_OSD_WHITE);

	if (_altitude_valid) {
		snprintf(text, sizeof(text), "%.1fm", (double)_altitude_m);
		_canvas.draw_text(left, status_line, text);
	}

	if (_armed) {
		const unsigned armed_s = static_cast<unsigned>(hrt_elapsed_time(&_armed_time) / 1_s);
		snprintf(text, sizeof(text), "%02u:%02u", armed_s / 60, armed_s % 60);
		_canvas.draw_text(right, status_line, text, Canvas::Align::Right);

	} else {
		_canvas.draw_text(right, status_line, "DISARMED", Canvas::Align::Right);
	}

	_canvas.draw_text(center, bottom_line, mode_util::nav_state_name(_nav_state, _vehicle_type), Canvas::Align::Center);
}

void FlexIOOSD::run()
{
	if (configure_overlay() != PX4_OK) {
		return;
	}

	hrt_abstime next_update = hrt_absolute_time();

	while (!should_exit()) {
		if (_parameter_update_sub.updated()) {
			parameter_update_s parameter_update;
			_parameter_update_sub.copy(&parameter_update);
			updateParams();
			configure_overlay();
		}

		follow_detected_standard();
		update_topics();
		render();
		up_flexio_osd_update(&_canvas.frame());

		// Fixed rate without drift, but never try to catch up on missed updates
		next_update += UPDATE_INTERVAL;
		const hrt_abstime now = hrt_absolute_time();

		if (next_update > now) {
			px4_usleep(next_update - now);

		} else {
			next_update = now;
		}
	}

	up_flexio_osd_deinit();
}

int FlexIOOSD::print_status()
{
	PX4_INFO("standard: %s (OSD_FLX_STD %li)", _standard == FLEXIO_OSD_STANDARD_PAL ? "PAL" : "NTSC",
		 (long)_param_osd_flx_std.get());
	up_flexio_osd_status();
	return 0;
}

FlexIOOSD *FlexIOOSD::instantiate(int argc, char *argv[])
{
	return new FlexIOOSD();
}

int FlexIOOSD::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return FlexIOOSD::instantiate(ac, av);
	}, argc, argv);
}

int FlexIOOSD::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("flexio_osd", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER, 1600,
					 (px4_main_t)&run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	desc.task_id = task_id;

	return PX4_OK;
}

int FlexIOOSD::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlexIOOSD::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OSD for analog PAL or NTSC video, generated by FlexIO on NXP i.MX RT boards.

A sync separator feeds the HSYNC input, and the OSD_EN and OSD_W outputs switch every pixel of a line to black,
white or the camera video. The overlay has 128x96 pixels and the task redraws it at 10 Hz with battery, altitude,
arming and flight mode. The board defines the pins in board_config.h, see drv_flexio_osd.h.

### Examples
```
flexio_osd start
```
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flexio_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flexio_osd_main(int argc, char *argv[])
{
	return ModuleBase::main(FlexIOOSD::desc, argc, argv);
}
