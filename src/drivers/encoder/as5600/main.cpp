#include "main.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#define POLL_RATE 800

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

void AS5600::print_usage()
{
	PRINT_MODULE_USAGE_NAME("as5600", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("encoder_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x36);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int encoder_main(int argc, char *argv[]){
	using ThisDriver = AS5600;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000; // 100kHz

	cli.i2c_address = 0x36;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator Iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr)
{
}

int AS5600::init(){
	if(I2C::init() != PX4_OK){
		return PX4_ERROR;
	}

	ScheduleOnInterval(POLL_RATE);
	_propellor_angle_pub.advertise();

	return PX4_OK;
}

uint8_t AS5600::readOneByte(uint8_t reg, uint8_t *data[]) {
	int ret = transfer(&reg, 1, &data, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
	}

	return ret;
}

uint8_t AS5600::readTwoBytes(uint8_t reg, uint8_t *data[]) {
	int ret = transfer(&reg, 1, &data, 2);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
	}

	return ret;
}

float AS5600::readAngle() {
	float angle = 0;

	uint8_t data[2];

	if (readTwoBytes(ANGLE_REG, &data) != PX4_OK){
		return PX4_ERROR;
	}


}

void AS5600::RunImpl(){
	int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);

	if (diffTime < POLL_RATE / 2) {
		PX4_ERR("as5600 loop calld to early");
		return;
	}

	_last_measurement_time = hrt_absolute_time();

	float angle = readAngle();
	float propeller_speed = (_last_angle_measurement - angle)/(_last_angle_measurement*1000000);

	_last_angle_measurement = angle;

	propellor_encoder_s propeller{};
	propeller.timestamp = _last_measurement_time;
	propeller.propeller_speed = propeller_speed;
	propeller.propeller_angle = angle;

	_propellor_angle_pub.publish(propeller);
}
