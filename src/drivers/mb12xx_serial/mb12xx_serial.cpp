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
#include <termios.h>

#include <px4_config.h>
#include <px4_defines.h>

#include <arch/board/board.h>
#include <board_config.h>
#include <drivers/device/device.h>

#define START_BYTE 'R'
#define DATA_LEN 3
#define FRAME_LEN (DATA_LEN + 2)
#define END_BYTE 13

class mb12xx_serial : public device::CDev
{
public:
	mb12xx_serial(const char *device);
	~mb12xx_serial();
	int open();
	int read();

	int _fd = -1;
private:
	const char *_device;
	uint8_t _buffer[10];
	uint8_t _buffer_index = 0;
};

extern "C" __EXPORT int mb12xx_serial_main(int argc, char *argv[]);

int mb12xx_serial_main(int argc, char *argv[])
{
	mb12xx_serial *dev = new mb12xx_serial("/dev/ttyS5");
	if (!dev) {
		PX4_ERR("Unable to allocate mb12xx_serial");
		return -1;
	}

	if (dev->open()) {
		PX4_ERR("Unable to open mb12xx_serial");
		delete dev;
		return -1;
	}

	struct pollfd fds[1];
	fds[0].fd = dev->_fd;
	fds[0].events = POLLIN;

	for (uint16_t i = 0; i < 25; i++) {
		int status = poll(fds, sizeof(fds) / sizeof(fds[0]), -1);
		if (status > 0) {
			printf("read()\n");
			dev->read();
		}
	}

	delete dev;

	return 0;
}

mb12xx_serial::mb12xx_serial(const char *device):
	CDev("mb12xx_serial", "/dev/mb12xx_serial"),
	_device(device)
{
}

mb12xx_serial::~mb12xx_serial()
{
	if (_fd > -1) {
		::close(_fd);
	}
}

int mb12xx_serial::open()
{
	_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_fd < 0) {
		PX4_ERR("Unable to open UART %s", _device);
		return -1;
	}

	struct termios config;

	int r = tcgetattr(_fd, &config);
	if (r) {
		PX4_ERR("Unable to get termios");
		goto error;
	}

	/* clear: data bit size, two stop bits, parity, flow control */
	config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CCTS_OFLOW | CRTS_IFLOW);
	/* set: 8 data bits, enable receiver, ignore modem status lines */
	config.c_cflag |= (CS8 | CREAD | CLOCAL);
	/* turn off output processing */
	config.c_oflag = 0;
	/* clear: echo, echo new line, canonical input and extended input */
	config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	r = cfsetispeed(&config, B9600);
	r |= cfsetospeed(&config, B9600);
	if (r) {
		PX4_ERR("Unable to set baudrate");
		goto error;
	}

	r = tcsetattr(_fd, TCSANOW, &config);
	if (r) {
		PX4_ERR("Unable to set termios");
		goto error;
	}

	return 0;

error:
	::close(_fd);
	_fd = -1;
	return -1;
}

int mb12xx_serial::read()
{
	int len = ::read(_fd, _buffer + _buffer_index, sizeof(_buffer) - _buffer_index);
	if (len < 1) {
		return len;
	}

	_buffer_index += len;

	for (uint8_t i = 0; i < _buffer_index; i++) {
		// look for the start byte
		if (_buffer[i] == START_BYTE) {
			// do it have the full frame size? the lsat byte is the end byte?
			if ((_buffer_index - i) >= FRAME_LEN && _buffer[i + FRAME_LEN - 1] == END_BYTE) {
				uint16_t distance = (_buffer[i + 1] - '0') * 100;
				distance += (_buffer[i + 2] - '0') * 10;
				distance += (_buffer[i + 3] - '0');

				printf("\n[%u (%u %u) (%u %u) (%u %u) %u] distance=%u\n", _buffer[i], _buffer[i + 1], _buffer[i + 1] - '0', _buffer[i + 2], _buffer[i + 2] - '0', _buffer[i + 3], _buffer[i + 3] - '0', _buffer[i + 4], distance);

				_buffer_index -= (i + FRAME_LEN);
				memmove(_buffer, &_buffer[i + FRAME_LEN], _buffer_index);
			} else {
				break;
			}
		}
	}

	// is buffer full of garbage? start over
	if (_buffer_index == sizeof(_buffer)) {
		_buffer_index = 0;
	}

	return 0;
}

