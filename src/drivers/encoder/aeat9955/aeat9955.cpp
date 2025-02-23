#include "main.hpp"


AEAT9955::AEAT9955(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_sample_perf(perf_alloc(PC_COUNT, MODULE_NAME": read")),
	_errors(perf_alloc(PC_COUNT, MODULE_NAME": err")),
	ready0_flag(false),
	ready1_flag(false),
	magnet_high_error_flag(false),
	magnet_low_error_flag(false),
	overvoltage_flag(false),
	undervolage_flag(false),
	memory_error_flag(false),
	tracker_error(false),
	_last_measurement_time(0),
	_last_angle_measurement(0.0),
	debug_data(0),
	debug_data2(0)
{
}

AEAT9955::~AEAT9955() {
	perf_free(_sample_perf);
	perf_free(_errors);

	delete _interface;
}

int AEAT9955::init(){

	start();

	return PX4_OK;
}

void AEAT9955::start()
{
	clear_alarm();

	/* start polling at the specified rate */
	ScheduleOnInterval((1264));

	_propellor_angle_pub.advertise();
}

void AEAT9955::startAutoCalibrate(){
	uint8_t buffer[2] = {0};

	buffer[1] = INIT_AUTO_CALIBRATION_VALUE;

	_interface->write(CALIBRATION_CONFIG_REG, buffer, 2);

	return;
}

uint8_t AEAT9955::stopAutoCalibrate(){
	uint8_t buffer[2] = {0};

	_interface->read(CALIBRATION_STATUS_REG, buffer, 2);

	uint8_t calibration_result = buffer[1];

	buffer[0] = 0;
	buffer[1] = EXIT_CALIBRATION_VALUE;
	_interface->write(CALIBRATION_CONFIG_REG, buffer, 2);

	return calibration_result;
}

void AEAT9955::readStatus(){
	uint8_t buffer[2] = {0};

	if(_interface->read(CHIP_STATUS_REG, buffer, 2) != PX4_OK){
		memory_error_flag = true;
		return;
	}

	debug_data2 = buffer[0] | buffer[1] << 8;

	magnet_high_error_flag = buffer[1] & 5;
	magnet_low_error_flag = buffer[1] & 4;

	memory_error_flag = buffer[1] & 1;
	tracker_error = buffer[1] & 0;
}

int AEAT9955::clear_alarm(){
	uint8_t buffer[2] = {0};

	buffer[1] = ALARM_CLEAR_VALUE;

	if(_interface->write(ALARM_CLEAR_REG, buffer, 2) != PX4_OK){
		return PX4_ERROR;
	}
	return PX4_OK;
}

void AEAT9955::print_status(){
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);

	::printf("Ready 0: %u\n", ready0_flag);
	::printf("Ready 1: %u\n", ready1_flag);
	::printf("Overvoltage Flag: %u\n", overvoltage_flag);
	::printf("Undervoltage Flag: %u\n", undervolage_flag);
	::printf("Magnet High Error Flag: %u\n", magnet_high_error_flag);
	::printf("Magnet Low Error Flag: %u\n", magnet_low_error_flag);
	::printf("Memory Error Flag: %u\n", memory_error_flag);
	::printf("Tracker Error: %u\n", tracker_error);
	::printf("Debug: %lu\n", debug_data);
	::printf("Debug2: %lu\n", debug_data2);
}

float AEAT9955::readAngle() {
	uint8_t buffer[3] = {0};
	if(_interface->read(POSITION_READ_REG, buffer, 3) != PX4_OK){
		return -1.0f;
	}
	uint32_t temp = buffer[2] | buffer[1] << 8 | (buffer[0] & 0x3F) << 16;

	debug_data = temp;

	float angle = 2.0f*3.14159f*((float)temp) / (1 << 22);	// Remove parity and error flag and devide by the resolution

	return angle;
}

void AEAT9955::RunImpl(){
	//int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);
	/*
	if (diffTime < POLL_RATE / 2) {
		PX4_ERR("AEAT9955 loop calld to early");
		return;
	}
	*/

	_last_measurement_time = hrt_absolute_time();

	float angle = readAngle();

	readStatus();
	//float propeller_speed = (_last_angle_measurement - angle)/(_last_angle_measurement*1000000);

	_last_angle_measurement = 0;

	propellor_encoder_s propellor{};
	propellor.timestamp = _last_measurement_time;
	propellor.propellor_speed = 0.0;
	propellor.propellor_angle = angle;

	_propellor_angle_pub.publish(propellor);
}
