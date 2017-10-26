/****************************************************************************
 *
 *   Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <poll.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>

#include <px4_config.h>
#include <px4_defines.h>

#include <arch/board/board.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_getopt.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/distance_sensor.h>

#define FRAME_START 0x5959

struct __attribute__((packed)) frame {
	uint16_t start;
	uint16_t distance_cm;
	uint16_t strength;
	uint8_t reserved;
	uint8_t quality;
	uint8_t checksum;
};

#define MIN_DISTANCE 0.30f
#define MAX_DISTANCE 12.0f

#define NAME "benewake_ulidar"
#define DEVICE_PATH "/dev/benewake_ulidar"

// 0.5sec
#define PROBE_USEC_TIMEOUT 500000

class benewake_ulidar : public device::CDev
{
public:
	benewake_ulidar(const char *device_path, const char *serial_port, uint8_t rotation);
	virtual ~benewake_ulidar();

	virtual int	init();
	void loop();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	int _fd = -1;
	const char *_serial_port;

	uint8_t _buffer[sizeof(struct frame) * 2];
	uint8_t _buffer_len = 0;

	uint8_t _rotation;

	orb_advert_t _topic = nullptr;

	ringbuffer::RingBuffer *_reports = nullptr;

	perf_counter_t _communication_error;

	int _read();
	void _publish(uint16_t distance);
};

extern "C" __EXPORT int benewake_ulidar_main(int argc, char *argv[]);

static void help()
{
	printf("missing command: try 'start' or 'test'\n");
	printf("options:\n");
	printf("    -d <serial port> to set the serial port were " NAME " is connected\n");
}

static void task_main_trampoline(int argc, char *argv[])
{
	const uint8_t rotation = (uint8_t)atoi(argv[2]);
	benewake_ulidar *inst = new benewake_ulidar(DEVICE_PATH, argv[1], rotation);

	if (!inst) {
		PX4_ERR("No memory to allocate " NAME);
		return;
	}

	if (inst->init()) {
		PX4_ERR("Unable to initialize " NAME);
		goto end;
	}

	inst->loop();

end:
	delete inst;
}

int benewake_ulidar_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *serial_port = "/dev/ttyS3";
	//uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *rotation = "8";

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;

		case 'r':
			rotation = myoptarg;
			break;

		default:
			help();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		help();
		return PX4_ERROR;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		const char *args[2] = { serial_port, rotation };

		px4_task_t r = px4_task_spawn_cmd(NAME "_thread", SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT, 1000, (px4_main_t)&task_main_trampoline,
				(char* const*)args);

		return r < 0 ? PX4_ERROR : PX4_OK;
	} else if (!strcmp(verb, "test")) {
		int fd = open(DEVICE_PATH, O_RDONLY);
		ssize_t sz;
		struct distance_sensor_s report;

		if (fd < 0) {
			PX4_ERR("Unable to open %s", DEVICE_PATH);
			return PX4_ERROR;
		}

		sz = read(fd, &report, sizeof(report));
		close(fd);

		if (sz != sizeof(report)) {
			PX4_ERR("No sample available in %s", DEVICE_PATH);
			return PX4_ERROR;
		}

		bool valid = false;
		if (report.current_distance > report.min_distance
				&& report.current_distance < report.max_distance) {
			valid = true;
		}
		printf("valid: %u\n", valid);
		printf("distance: %0.3fm\n", (double)report.current_distance);
		printf("time: %llu\n", report.timestamp);
	} else {
		help();
		return PX4_ERROR;
	}

	return PX4_OK;
}

benewake_ulidar::benewake_ulidar(const char *device_path, const char *serial_port, uint8_t rotation):
	CDev(NAME, device_path),
	_serial_port(serial_port),
	_rotation(rotation),
	_communication_error(perf_alloc(PC_COUNT, "benewake_ulidar_uart_communication_errors"))
{
}

benewake_ulidar::~benewake_ulidar()
{
	if (_fd > -1) {
		::close(_fd);
	}

	if (_reports) {
		delete _reports;
	}

	if (_topic) {
		orb_unadvertise(_topic);
	}

	perf_free(_communication_error);
}

int benewake_ulidar::init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize device\n");
		return -1;
	}

	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));
	if (!_reports) {
		PX4_ERR("No memory to allocate RingBuffer");
		return -1;
	}

	_fd = ::open(_serial_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (_fd < 0) {
		PX4_ERR("Unable to open serial port");
		return -1;
	}

	struct termios config;

	int r = tcgetattr(_fd, &config);
	if (r) {
		PX4_ERR("Unable to get termios");
		return -1;
	}

	/* clear: data bit size, two stop bits, parity, flow control */
	config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CCTS_OFLOW | CRTS_IFLOW);
	/* set: 8 data bits, enable receiver, ignore modem status lines */
	config.c_cflag |= (CS8 | CREAD | CLOCAL);
	/* turn off output processing */
	config.c_oflag = 0;
	/* clear: echo, echo new line, canonical input and extended input */
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN);

	r = cfsetispeed(&config, B115200);
	r |= cfsetospeed(&config, B115200);
	if (r) {
		PX4_ERR("Unable to set baudrate");
		return -1;
	}

	r = tcsetattr(_fd, TCSANOW, &config);
	if (r) {
		PX4_ERR("Unable to set termios");
		return -1;
	}

	hrt_abstime timeout_usec, now;
	for (now = hrt_absolute_time(), timeout_usec = now + PROBE_USEC_TIMEOUT;
		 now < timeout_usec;
		 now = hrt_absolute_time()) {

		if (_read() > 0) {
			return 0;
		}

		usleep(1000);
	}

	PX4_ERR("No readings from " NAME);

	return -1;
}

void benewake_ulidar::loop()
{
	pollfd fds[1] = {};

	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	while (true) {
		if (::poll(fds, (sizeof(fds) / sizeof(fds[0])), -1) < 1) {
			continue;
		}

		if (!(fds[0].revents & POLLIN)) {
			continue;
		}

		_read();
	}
}

void benewake_ulidar::_publish(uint16_t distance_cm)
{
	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = ((float)distance_cm / 100.0f);
	report.min_distance = MIN_DISTANCE;
	report.max_distance = MAX_DISTANCE;
	report.covariance = 0.0f;
	report.id = 0;

	_reports->force(&report);

	if (_topic == nullptr) {
		_topic = orb_advertise(ORB_ID(distance_sensor), &report);
	} else {
		orb_publish(ORB_ID(distance_sensor), _topic, &report);
	}
}

int benewake_ulidar::_read()
{
	int ret = ::read(_fd, _buffer + _buffer_len, sizeof(_buffer) - _buffer_len);
	if (ret < 1) {
		return ret;
	}

	_buffer_len += ret;
	ret = 0;

	for (uint8_t i = 0; i < _buffer_len; i++) {
		// there is a full frame in the buffer?
		if ((unsigned)(_buffer_len - i) < sizeof(struct frame)) {
			break;
		}

		struct frame *frame = (struct frame *)(_buffer + i);

		// found the frame start?
		if (frame->start == FRAME_START) {
			uint8_t checksum_calc = 0;

			for (uint8_t j = 0; j < (sizeof(struct frame) - 1); j++) {
				checksum_calc += _buffer[i + j];
			}

			if (checksum_calc != frame->checksum) {
				perf_count(_communication_error);
				continue;
			}

			// datasheet says the output is cm but it is mm
			frame->distance_cm /= 10;

			// return how many frames were read
			ret++;
			_publish(frame->distance_cm);

			// debug remove it
			static uint8_t a = 0;
			uint8_t low, high;
			low = frame->distance_cm;
			high = (frame->distance_cm >> 8);
			if (a == 50) {
				printf("low=%u high=%u distance_cm=%u distance_m=%0.3f quality=%u strength=%u\n", low, high, frame->distance_cm, (double)(frame->distance_cm / 100.0f), frame->quality, frame->strength);
				a = 0;
			} else {
				a++;
			}

			_buffer_len -= (i + sizeof(struct frame));
			memmove(_buffer, _buffer + (i + sizeof(struct frame)), _buffer_len);
			i = 0;
		}
	}

	// is buffer full of garbage? start over
	if (_buffer_len == sizeof(_buffer)) {
		_buffer_len = 0;
		perf_count(_communication_error);
	}

	return ret;
}

ssize_t benewake_ulidar::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			rbuf++;
		}
	}

	return ret ? ret : -EAGAIN;
}

int benewake_ulidar::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return -1;
}
