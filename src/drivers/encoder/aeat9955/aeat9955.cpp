#include "main.hpp"


AEAT9955::AEAT9955(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_sample_perf(perf_alloc(PC_COUNT, MODULE_NAME": read")),
	_errors(perf_alloc(PC_COUNT, MODULE_NAME": err")),
	magnet_high_error_flag(false),
	magnet_low_error_flag(false),
	memory_error_flag(false),
	tracker_error(false),
	ready_flag(false),
	_last_measurement_time(0),
	_last_angle_measurement(0.0)
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
	/* start polling at the specified rate */
	ScheduleOnInterval((1000000));

	_propellor_angle_pub.advertise();
}

void AEAT9955::readStatus(){
	uint8_t buffer[2];

	if(_interface->read(CHIP_STATUS, buffer, 2) != PX4_OK){
		return;
	}

	magnet_high_error_flag = buffer[0] & 5;
	magnet_low_error_flag = buffer[0] & 4;

	memory_error_flag = buffer[0] & 1;
	tracker_error = buffer[0] & 0;
}

void AEAT9955::print_status(){
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);

	::printf("Magnet High Error Flag: %u", magnet_high_error_flag);
	::printf("Magnet Low Error Flag: %u", magnet_low_error_flag);
	::printf("Memory Error Flag: %u", memory_error_flag);
	::printf("Tracker Error: %u", tracker_error);
}

float AEAT9955::readAngle() {
	uint8_t buffer[3];
	if(_interface->read(POSITION_READ, buffer, 3) != PX4_OK){
		memory_error_flag = true;
		return -0.5;
	}
	uint32_t temp = buffer[0] | buffer[1] << 8 | buffer[2] << 16;
	float angle = ((float) (temp & 0x3FF)) / (1 << 18);	// Remove parity and error flag and devide by the resolution

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
	//float propeller_speed = (_last_angle_measurement - angle)/(_last_angle_measurement*1000000);

	_last_angle_measurement = angle;

	propellor_encoder_s propellor{};
	propellor.timestamp = _last_measurement_time;
	propellor.propellor_speed = 0.0;
	propellor.propellor_angle = angle;

	_propellor_angle_pub.publish(propellor);
}
