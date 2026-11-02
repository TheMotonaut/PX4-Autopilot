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
 * @file FlexIOOSD.hpp
 *
 * Analog video OSD drawn by the FlexIO overlay of NXP i.MX RT boards.
 */

#pragma once

#include "Canvas.hpp"

#include <drivers/drv_flexio_osd.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class FlexIOOSD : public ModuleBase, public ModuleParams
{
public:
	static Descriptor desc;

	FlexIOOSD();
	~FlexIOOSD() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static int run_trampoline(int argc, char *argv[]);

	static FlexIOOSD *instantiate(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static constexpr hrt_abstime UPDATE_INTERVAL{100_ms};	// 10 Hz

	// OSD_FLX_STD values
	static constexpr int32_t STANDARD_AUTO = 0;
	static constexpr int32_t STANDARD_PAL = 1;
	static constexpr int32_t STANDARD_NTSC = 2;

	/**
	 * (Re)start the overlay for the configured or detected video standard
	 */
	int configure_overlay();

	/**
	 * Restart the overlay when auto detection finds a different standard than it is timed for
	 */
	void follow_detected_standard();

	void update_topics();
	void render();

	Canvas _canvas{};

	flexio_osd_standard_t _standard{FLEXIO_OSD_STANDARD_PAL};

	bool _battery_valid{false};
	float _battery_voltage_v{0.f};
	float _battery_remaining{-1.f};

	bool _altitude_valid{false};
	float _altitude_m{0.f};

	bool _armed{false};
	hrt_abstime _armed_time{0};
	uint8_t _nav_state{0};
	uint8_t _vehicle_type{0};

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_FLX_STD>) _param_osd_flx_std,
		(ParamFloat<px4::params::OSD_FLX_H_OFS>) _param_osd_flx_h_ofs,
		(ParamInt<px4::params::OSD_FLX_V_OFS>) _param_osd_flx_v_ofs
	)
};
