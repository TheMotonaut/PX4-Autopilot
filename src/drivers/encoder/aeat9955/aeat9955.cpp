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
	_last_angle_measurement(0.0),
	debug_data(0),
	debug_data2(0),
	data0(0),
	data1(0),
	data2(0)
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
	ScheduleOnInterval((1000000));

	_propellor_angle_pub.advertise();
}

void AEAT9955::readStatus(){
	uint8_t buffer[2] = {0};

	if(_interface->read(CHIP_STATUS, buffer, 2) != PX4_OK){
		memory_error_flag = true;
		return;
	}

	debug_data2 = buffer[0] | buffer[1] << 8;

	magnet_high_error_flag = buffer[0] & 5;
	magnet_low_error_flag = buffer[0] & 4;

	memory_error_flag = buffer[0] & 1;
	tracker_error = buffer[0] & 0;
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

	::printf("Magnet High Error Flag: %u", magnet_high_error_flag);
	::printf("Magnet Low Error Flag: %u", magnet_low_error_flag);
	::printf("Memory Error Flag: %u", memory_error_flag);
	::printf("Tracker Error: %u", tracker_error);
	::printf(" debug: %lu", debug_data);
	::printf(" debug2: %lu", debug_data2);
	::printf("\ndata0 :%u", data0);
	::printf("\ndata1: %u", data1);
	::printf("\ndata2: %u\n", data2);
}

float AEAT9955::readAngle() {
	uint8_t buffer[3] = {0};
	if(_interface->read(POSITION_READ, buffer, 3) != PX4_OK){
		memory_error_flag = true;
		return -0.5;
	}
	uint32_t temp = buffer[2] | buffer[1] << 8 | (buffer[0] & 0x3F) << 16;

	data0 = (buffer[0] & 0x3F);
	data1 = buffer[1];
	data2 = buffer[2];

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
	angle = readAngle();

	readStatus();
	//float propeller_speed = (_last_angle_measurement - angle)/(_last_angle_measurement*1000000);

	_last_angle_measurement = 0;

	propellor_encoder_s propellor{};
	propellor.timestamp = _last_measurement_time;
	propellor.propellor_speed = 0.0;
	propellor.propellor_angle = angle;

	_propellor_angle_pub.publish(propellor);
}
