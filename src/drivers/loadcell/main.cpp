#include "main.hpp"

#include <lib/drivers/device/Device.hpp>

#define CLEAR_ALARM_CUSTOM_METHOD 0
#define START_AUTO_CALIBRATION_CUSTOM_METH0D 1
#define STOP_AUTO_CALIBRATION_CUSTOM_METHOD 2

//__EXPORT int px4_simple_app_main(int argc, char *argv[]);

LOADCELL::LOADCELL(const char *port) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	received(0),
	bajs(0),
	bajs2(0),
	bajs3(0)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;

	device_id.devid_s.devtype = DRV_LOAD_LOADCELL;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

}

LOADCELL::~LOADCELL(){
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int LOADCELL::init(){
	int ret = 0;

	do {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		unsigned speed = B115200;

		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		uart_config.c_oflag &= ~ONLCR;

		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while(0);

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}
void LOADCELL::start() {
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);

	_loadcell_pub.advertise();
}

void LOADCELL::stop() {
	ScheduleClear();
}

void LOADCELL::status() {
	perf_print_counter(_sample_perf);
	PX4_INFO("Bytes: %i", received);
	PX4_INFO("bajs1: %i", bajs);
	PX4_INFO("Bytes2: %i", bajs2);
	PX4_INFO("Bytes3: %i", bajs3);
	perf_print_counter(_comms_errors);

}

void LOADCELL::Run() {
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(7_ms, 87 * 9);
		return;
	}
}

int LOADCELL::collect() {
	perf_begin(_sample_perf);

	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	char readbuf[sizeof(_linebuf)] {};

	unsigned readlen = sizeof(readbuf) - 1;

	int ret = 0;
	int measurement = -404;

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if(!bytes_available) {
		perf_end(_sample_perf);

		received++;
		return 0;
	}

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		ret = ::read(_fd, &readbuf[0], readlen);

		bajs++;

		if (ret < 0) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		bajs2++;

		_last_read = hrt_absolute_time();

		for (int i = 0; i < ret; i++) {
			if (i > 5){
				tcflush(_fd, TCIFLUSH);
				return -EAGAIN;
			}
			if(readbuf[i] == 0xFF) {
				measurement = readbuf[i + 1] << 0 |readbuf[i + 2] << 8 | readbuf[i + 3] << 16 | readbuf[i + 4] << 24;
			}
		}

		bytes_available -= ret;
	} while (bytes_available > 0);

	bajs3++;

	loadcell_s measurement_msg{};
	measurement_msg.timestamp = timestamp_sample;
	measurement_msg.value = measurement;

	_loadcell_pub.publish(measurement_msg);

	perf_end(_sample_perf);

	return PX4_OK;
}

namespace loadcell{
	LOADCELL *g_dev{nullptr};

	int start(const char *port, uint8_t rotation);
	int status();
	int stop();
	int usage();

	int start(const char *port){
		if (g_dev != nullptr) {
			PX4_ERR("already started");
			return PX4_OK;
		}

		g_dev = new LOADCELL(port);

		if (g_dev == nullptr) {
			PX4_ERR("driver start failed");
			return PX4_ERROR;
		}

		if (OK != g_dev->init()) {
			PX4_ERR("driver start failed");
			delete g_dev;
			g_dev = nullptr;
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	int status(){
		if (g_dev == nullptr) {
			PX4_ERR("driver not running");
			return 1;
		}

		printf("state @ %p\n", g_dev);
		g_dev->status();
		return 0;
	}
	int stop() {
		if (g_dev != nullptr) {
			PX4_INFO("stopping driver");
			delete g_dev;
			g_dev = nullptr;
			PX4_INFO("driver stopped");

		} else {
			PX4_ERR("driver not running");
			return 1;
		}

		return PX4_OK;
	}
	int usage() {
		PRINT_MODULE_USAGE_NAME("loadcell", "driver");
		PRINT_MODULE_USAGE_SUBCATEGORY("loadcell");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

		return PX4_OK;
	}
}

extern "C" __EXPORT int loadcell_main(int argc, char *argv[]){
	const char *device_path = LOADCELL_DEFAULT_PORT;

	int myoptind = 1;

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return loadcell::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return loadcell::start(device_path);


	} else if (!strcmp(argv[myoptind], "stop")) {
		return loadcell::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return loadcell::status();
	}


	loadcell::usage();

	return PX4_ERROR;
}


