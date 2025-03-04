/**
 * @file spi.cpp
 *
 * Driver for the Broadcom AEAT9955 connected via SPI.
 *
 * @author Simon Björklund
 */


#include <drivers/device/spi.h>
#include "main.hpp"

device::Device *AEAT9955_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);

class AEAT9955_SPI : public device::SPI{
	public:
		AEAT9955_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);
		~AEAT9955_SPI() override = default;

		virtual int init();

		/**
		 * Read directly from the device.
		 *
		 * The actual size of each unit quantity is device-specific.
		 *
		 * @param reg	The register address at which to start reading
		 * @param data	The buffer into which the read values should be placed.
		 * @param count	The number of items to read.
		 * @return		The number of items read on success, negative errno otherwise.
		 */
		int	read(unsigned reg, void *data, unsigned count) override;

		/**
		 * Write directly to the device.
		 *
		 * The actual size of each unit quantity is device-specific.
		 *
		 * @param reg	The register address at which to start writing.
		 * @param data	The buffer from which values should be read.
		 * @param count	The number of items to write.
		 * @return		The number of items written on success, negative errno otherwise.
		 */
		int	write(unsigned reg, void *data, unsigned count) override;

		/**
		 * Read a register from the device.
		 *
		 * @param		The register to read.
		 * @return		The value that was read.
		 */

	protected:
		int probe() override;

};

device::Device *AEAT9955_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode){
	return new AEAT9955_SPI(bus, chip_select, bus_frequency, spi_mode);
}

AEAT9955_SPI::AEAT9955_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode) : SPI(DRV_SENS_DEVTYPE_AEAT9955, MODULE_NAME, bus, chip_select, spi_mode, bus_frequency){
}

int AEAT9955_SPI::init(){
	int ret;
	ret = SPI::init();

	if (ret != PX4_OK){
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	return PX4_OK;
}

int AEAT9955_SPI::probe(){
	uint8_t whoami = 0;

	bool success = 1;

	DEVICE_DEBUG("AEAT9955_SPI::probe %s, whoami: 0x%02x", (success ? "Succeded" : "fauled"), whoami);
	return success? OK: -EIO;
}

void calc_parity(uint8_t *data){

	uint32_t temp = data[0] | data[1] << 8 | data[2] << 16;

	uint8_t set_bits = popcount(temp);
	/*
	while(temp) {
		if(temp & 0x1){
			set_bits += 1;
		}
		temp = temp >> 1;
	}
	*/



	data[1] = ((set_bits % 2) << 7) | data[1];
}

int AEAT9955_SPI::write(unsigned reg, void *data, unsigned count){
	uint8_t buf[3] = {0};

	buf[1] = reg;

	calc_parity(buf);

	transfer(&buf[0], nullptr, count);

	uint8_t buf2[3] = {0};

	memcpy(&buf2[0], data, count);
	transfer(&buf2[0], nullptr, count);

	return PX4_OK;
}

int AEAT9955_SPI::read(unsigned reg, void *data, unsigned count){
	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};

	tx[1] = reg;
	tx[0] = (1 << 6);

	calc_parity(tx);

	transfer(&tx[0], nullptr, count);
	int ret = transfer(nullptr, &rx[0], count);
	memcpy(data, &rx[0], count);
	return ret;
}
