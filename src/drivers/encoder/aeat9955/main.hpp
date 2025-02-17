
#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/perf/perf_counter.h>

#include <drivers/device/Device.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/propellor_encoder.h>

#define POSITION_READ 0x3F


// Volatile Memory address
#define CHIP_STATUS 0x29

#define UNLOCK_LEVEL1_ADDR 0x10
#define UNLOCK_LEVEL1_CODE 0xAB
#define LOAD_PAGE_TO_MTP 0x16

#define POLL_RATE 100

device::Device *AEAT9955_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);

class AEAT9955 : public I2CSPIDriver<AEAT9955> {
	public:
		AEAT9955(device::Device *interface, const I2CSPIDriverConfig &config);
		virtual ~AEAT9955();

		static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
		static void print_usage();

		int init();
		void print_status() override;
		void RunImpl();

	protected:
		int probe();

	private:
		device::Device *_interface;

		uORB::Publication<propellor_encoder_s> _propellor_angle_pub{ORB_ID(propellor_encoder)};

		perf_counter_t _sample_perf;
		perf_counter_t _errors;

		bool magnet_high_error_flag;
		bool magnet_low_error_flag;

		bool memory_error_flag;
		bool tracker_error;

		bool ready_flag;

		uint64_t _last_measurement_time;
		float _last_angle_measurement;

		void readStatus();
		float readAngle();

		void start();
};





