
#pragma once

#include <px4_platform_common/log.h>
#include <lib/perf/perf_counter.h>
#include <uORB/topics/PropellorEncoder.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/propellor_encoder.h>
#include <drivers/drv_hrt.h>

#define ANGLE_REG 0x0E
#define RAW_ANGLE_REG 0x0C
#define STATUS_REG 0x0B

class AS5600 : public device::I2C, public ModuleParams, public I2CSPIDriver<PCF8583> {
	public:
		AS5600(const I2CSPIDriverConfig &config);
		~AS5600() override = default;

		static void print_usage();
		void RunImpl();
		void print_status() override;

	private:
		int probe() override;

		uint8_t readOneByte(uint8_t reg);
		uint8_t readTwoBytes(uint8_t reg);

		void setRegister(uint8_t reg, uint8_t, value);

		float _last_angle_measurement;
		uint64_t _last_measurement_time;

		uORB::Publication<propellor_encoder_s> _propellor_angle_pub{ORB_ID(propellor_encoder)};

};
