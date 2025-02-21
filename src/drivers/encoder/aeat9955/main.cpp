#include "main.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>


//__EXPORT int px4_simple_app_main(int argc, char *argv[]);

extern "C" __EXPORT int aeat9955_main(int argc, char *argv[]);

void AEAT9955::print_usage()
{
	PRINT_MODULE_USAGE_NAME("AEAT9955", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("encoder_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *AEAT9955::instantiate(const I2CSPIDriverConfig &config, int runtime_instance){
	device::Device *interface = nullptr;

	interface = AEAT9955_SPI_interface(config.bus, config.spi_devid, config.bus_frequency, config.spi_mode);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != PX4_OK){
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", config.bus, config.spi_devid);
		return nullptr;
	}

	AEAT9955 *dev = new AEAT9955(interface, config);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

void AEAT9955::custom_method(const BusCLIArguments &cli){
	clear_alarm();
}


extern "C" int aeat9955_main(int argc, char *argv[]){
	using ThisDriver = AEAT9955;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 2000000; // 1MHz

	cli.spi_mode = SPIDEV_MODE1;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_AEAT9955);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	} else if (!strcmp(verb, "clear")) {
		return ThisDriver::module_custom_method(cli, iterator);
	}
	ThisDriver::print_usage();

	return PX4_ERROR;
}


