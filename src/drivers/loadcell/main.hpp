
#pragma once
#include <drivers/drv_hrt.h>

#include <termios.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/loadcell.h>

#define LOADCELL_DEFAULT_PORT	"/dev/ttyS6"

using namespace time_literals;


class LOADCELL : public  px4::ScheduledWorkItem {
	public:
		LOADCELL(const char *port);
		virtual ~LOADCELL();

		int init();
		void RunImpl();
		void status();


	private:
		int received;
		int bajs;
		int bajs2;
		int bajs3;
		int collect();

		void Run() override;

		void start();
		void stop();

		uORB::Publication<loadcell_s> _loadcell_pub{ORB_ID(loadcell)};

		char _linebuf[8] {};
		char _port[20] {};

		int _fd{-1};

		hrt_abstime _last_read{0};

		static constexpr int kCONVERSIONINTERVAL{9_ms};

		perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
		perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};





