#define _GNU_SOURCE
#include <features.h>

#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/select.h>


#define LINE_BUFFER_SIZE 1024
#define INVERTER_HID_BUFFER_SIZE 8
#define INVERTER_TIMEOUT_USEC 20000

#define LOG_TRACE(format, ...) do { if(trace) { LOG(format, ##__VA_ARGS__); } } while (0)
#define LOG_DEBUG(format, ...) do { if(debug) { LOG(format, ##__VA_ARGS__); } } while (0)
#define LOG_ERROR(format, ...) do { if(!silent) { LOG(format, ##__VA_ARGS__); } } while (0)

#define FLAG_ZERO_TERM		0x01
#define FLAG_READ_HID_BLOCK	0x02

static volatile uint8_t signal_exit = 0;
static uint8_t trace = 0;
static uint8_t debug = 0;
static uint8_t silent = 0;

static const char *device = NULL;
static const char *pty = NULL;
static char separator = '\n';
static uint8_t hid = 0;

static char input_buffer[LINE_BUFFER_SIZE];
static size_t input_buffer_cursor = 0;

static char inverter_read_buffer[LINE_BUFFER_SIZE];
static char inverter_write_buffer[INVERTER_HID_BUFFER_SIZE];
static size_t inverter_read_buffer_cursor = 0;

static void LOG(const char *format, ...) {
	va_list ap;

	char fmt[2048];
	snprintf(fmt, sizeof(fmt), "%s\n", format);

	va_start(ap, format);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
}

static int usage(int argc, const char *argv[]) {
	fprintf(stderr, "usage: %s [-D|--debug|-T|--trace|-S|--silent] [-H|--hid] [-z|--zero] [-p PTY|--pty=PTY] [--] DEVICE\n", argv[0]);
	return 1;
}

static ssize_t read_line(int fd, unsigned char *buffer, size_t size, size_t *cursor, uint8_t flags) {
	ssize_t bsize = flags & FLAG_READ_HID_BLOCK ? INVERTER_HID_BUFFER_SIZE : 1;
	unsigned char block[bsize];

	for (size_t pending = size - *cursor ; pending > 0 ; pending -= bsize) {
		ssize_t rd;

		rd = read(fd, block, bsize);

		if (!rd || rd < 0 && errno == EAGAIN) {
			return 0;
		}

		if (rd < 0) {
			return rd;
		}

		if (rd != bsize) {
			LOG_ERROR("Incomplete block received");
			return -1;
		}

		for (size_t n = 0 ; n < bsize ; n++) {
			unsigned char c = block[n];

			LOG_TRACE("Read%s%c%s[%02x]", c < 32 || c > 127 ? "" : " ", c < 32 || c > 127 ? ' ' : c,  c < 32 || c > 127 ? "" : " ", c);

			if ((flags & FLAG_ZERO_TERM) && !c || c == '\r' || c == '\n') {
				if (!*cursor) {
					continue;
				}

				size = *cursor;
				*cursor = 0;

				buffer[size] = 0;

				return size;
			}

			buffer[(*cursor)++] = c;
		}
	}

	return -1;
}

static uint16_t inverter_cal_crc_half(const unsigned char *data, size_t len) {
	static const uint16_t crc_ta[16]= {
		0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
		0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef
	};

	uint16_t crc;
	uint8_t da;
	const uint8_t *ptr;
	uint8_t bCRCHign;
	uint8_t bCRCLow;

	ptr = data;
	crc = 0;

	while (len--) {
		da = ((uint8_t)(crc>>8))>>4;
		crc <<= 4;
		crc ^= crc_ta[da^(*ptr>>4)];
		da = ((uint8_t)(crc>>8))>>4;
		crc <<= 4;
		crc ^= crc_ta[da^(*ptr&0x0f)];
		ptr++;
	}

	bCRCLow = crc;
	bCRCHign = (crc>>8);

	if (bCRCLow == 0x28 || bCRCLow == 0x0d || bCRCLow == 0x0a) bCRCLow++;
	if (bCRCHign == 0x28 || bCRCHign == 0x0d || bCRCHign == 0x0a) bCRCHign++;

	crc = ((uint16_t)bCRCHign)<<8;
	crc += bCRCLow;

	return crc;
}

static uint8_t inverter_check_crc(const unsigned char *data, size_t len) {
	if (len < 2) {
		return 0;
	}

	uint16_t crc = inverter_cal_crc_half(data, len-2);

	return data[len-2] == (crc>>8) && data[len-1] == (crc&0xff);
}

static ssize_t inverter_write(int fd, const unsigned char * cmd) {
	size_t size = strlen(cmd);
	uint16_t crc = inverter_cal_crc_half((uint8_t*) cmd, size);
	unsigned char pkt[3];

	pkt[0] = crc >> 8;
	pkt[1] = crc & 0xff;
	pkt[2] = 0x0d;

	LOG_TRACE("Writing: '%s' [%02x] [%02x] [%02x]", cmd, pkt[0], pkt[1], pkt[2]);

	for (ssize_t cursor = 0, pending = size ; pending > -3 ; ) {
		size_t wr;

		for (wr = 0 ; wr < INVERTER_HID_BUFFER_SIZE ; wr++) {
			char c;

			if (pending > 0) {
				c = cmd[cursor];
				cursor++;
				pending--;
			}
			else if (pending > -3) {
				c = pkt[-pending];
				pending--;
			}
			else {
				c = 0;
			}

			inverter_write_buffer[wr] = c;
		}

		if (write(fd, inverter_write_buffer, wr) != wr) {
			return -1;
		}
	}

	return 0;
}

static char * inverter_read(unsigned char * data, size_t size) {
	if (size < 3) {
		LOG_DEBUG("Bad frame length (%u)", size);
		return NULL;
	}

	if (data[0] != '(') {
		unsigned char t = data[size - 2];
		data[size - 2] = 0;
		LOG_DEBUG("Bad frame received: '%s' [%02x] [%02x]", data, t, data[size - 1]);
		return NULL;
	}

	if (!(inverter_check_crc(data, size))) {
		unsigned char t = data[size - 2];
		data[size - 2] = 0;
		LOG_DEBUG("Bad frame CRC: '%s' [%02x] [%02x]", data + 1, t, data[size - 1]);
		return NULL;
	}

	data[size - 2] = 0;

	return data + 1;
}

static void inverter_close(int fd) {
	close(fd);
}

static int inverter_pty(const char * name) {
	int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		LOG_ERROR("Unable to allocate pty. %s (%d)", strerror(errno), errno);
		return -1;
	}

	const char * ptname = ptsname(fd);

	if (!ptname) {
		LOG_ERROR("Unable to get pty name. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	LOG_DEBUG("Allocated PTY '%s'.", ptname);

	struct termios settings;

	int rc = tcgetattr(fd, &settings);

	if (rc) {
		LOG_ERROR("PTY setup failed. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	// Serial terminal settings (2400 8N1, raw mode)

	cfsetospeed(&settings, B2400);	// output baud rate
	cfsetispeed(&settings, B2400);	// input baud rate same as output
	settings.c_cflag &= ~PARENB;	// no parity
	settings.c_cflag &= ~CSTOPB;	// 1 stop bit
	settings.c_cflag &= ~CSIZE;	// character size mask
	settings.c_cflag |= CS8;	// 8 bits
	settings.c_cflag |= CLOCAL;	// ignore modem control lines
	settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);	// raw mode
	settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON); // raw mode
	settings.c_oflag &= ~OPOST;	// raw mode

	rc = tcsetattr(fd, TCSANOW, &settings);

	if (rc) {
		LOG_ERROR("PTY setup failed'. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	rc = grantpt(fd);

	if (rc < 0) {
		LOG_ERROR("Unable to set pty permissions. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	rc = unlockpt(fd);

	if (rc < 0) {
		LOG_ERROR("Unable to unlock pty. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	rc = ioctl(fd, TIOCGPTPEER, O_RDWR | O_NOCTTY);

	if (rc < 0) {
		LOG_ERROR("PTY setup failed. %s (%d)", strerror(errno), errno);
		close(fd);
		return -1;
	}

	rc = symlink(ptname, name);

	if (rc < 0) {
		LOG_ERROR("Unable to link '%s' to pty. %s (%d)", name, strerror(errno), errno);
		close(fd);
		return -1;
	}

	LOG_DEBUG("Created PTY '%s'.", name);

	return fd;
}

static int inverter_open(const char * device) {
	int fd = open(device, O_RDWR | O_NONBLOCK);

	if (fd < 0) {
		LOG_ERROR("Unable to open device '%s'. %s (%d)", device, strerror(errno), errno);
		return fd;
	}

	if (flock(fd, LOCK_EX | LOCK_NB)) {
		LOG_ERROR("Device '%s' busy. %s (%d)", device, strerror(errno), errno);
		return -1;
	}

	if (!hid) {
		struct termios settings;

		int rc = tcgetattr(fd, &settings);

		if (rc) {
			LOG_ERROR("Serial setup failed for '%s'. %s (%d)", device, strerror(errno), errno);
			inverter_close(fd);
			return -1;
		}

		// Serial terminal settings (2400 8N1, raw mode)

		cfsetospeed(&settings, B2400);	// output baud rate
		cfsetispeed(&settings, B2400);	// input baud rate same as output
		settings.c_cflag &= ~PARENB;	// no parity
		settings.c_cflag &= ~CSTOPB;	// 1 stop bit
		settings.c_cflag &= ~CSIZE;	// character size mask
		settings.c_cflag |= CS8;	// 8 bits
		settings.c_cflag |= CLOCAL;	// ignore modem control lines
		settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);	// raw mode
		settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON); // raw mode
		settings.c_oflag &= ~OPOST;	// raw mode

		rc = tcsetattr(fd, TCSANOW, &settings);

		if (rc) {
			LOG_ERROR("Serial setup failed for '%s'. %s (%d)", device, strerror(errno), errno);
			inverter_close(fd);
			return -1;
		}

/*
		rc = tcflush(fd, TCIOFLUSH);

		if (rc) {
			LOG_ERROR("Error flushing serial device '%s'. %s (%d)", device, strerror(errno), errno);
			inverter_close(fd);
			return -1;
		}
*/
	}

	return fd;
}

static void signal_handler(int sig) {
	LOG_DEBUG("Exitting from signal %u.", sig);
	signal_exit = 1;
}

int main(int argc, const char *argv[]) {
	for (int n = 1 ; n < argc ; n++) {
		if (argv[n][0] != '-' || n > 1 && !strcmp(argv[n - 1], "--")) {
			if (n != argc - 1) {
				return usage(argc, argv);
			}

			device = argv[n];
			break;
		}
		else if (!strcmp(argv[n], "-T") || !strcmp(argv[n], "--trace")) {
			if (silent || debug || trace) {
				return usage(argc, argv);
			}

			trace = 1;
			debug = 1;
		}
		else if (!strcmp(argv[n], "-D") || !strcmp(argv[n], "--debug")) {
			if (silent || debug || trace) {
				return usage(argc, argv);
			}

			debug = 1;
		}
		else if (!strcmp(argv[n], "-S") || !strcmp(argv[n], "--silent")) {
			if (silent || debug || trace) {
				return usage(argc, argv);
			}

			silent = 1;
		}
		else if (!strcmp(argv[n], "-z") || !strcmp(argv[n], "--zero")) {
			if (!separator) {
				return usage(argc, argv);
			}

			separator = 0;
		}
		else if (!strcmp(argv[n], "-H") || !strcmp(argv[n], "--hid")) {
			if (hid) {
				return usage(argc, argv);
			}

			hid = 1;
		}
		else if (!strcmp(argv[n], "-p")) {
			if (pty || ++n >= argc ) {
				return usage(argc, argv);
			}

			pty = argv[n];
		}
		else if (!strncmp(argv[n], "--pty=", 6)) {
			if (pty || !argv[n][6]) {
				return usage(argc, argv);
			}

			pty = argv[n] + 6;
		}
		else if (strcmp(argv[n], "--")) {
			return usage(argc, argv);
		}
	}

	if (!device || !*device) {
		return usage(argc, argv);
	}

	struct sigaction signals = {0};

	signals.sa_handler = SIG_IGN;
	sigaction(SIGINT, &signals, NULL);
	sigaction(SIGTERM, &signals, NULL);
	sigaction(SIGUSR1, &signals, NULL);
	sigaction(SIGUSR2, &signals, NULL);
	sigaction(SIGHUP, &signals, NULL);

	int fd = inverter_open(device);

	if (fd < 0) {
		return 2;
	}

	int input = 0;
	int output = 1;

	if (pty) {
		input = output = inverter_pty(pty);
		if (input < 0){
			inverter_close(fd);
			return 10;
		}
	}

	int select_fds = (fd > input ? fd : input) + 1;

	signals.sa_handler = signal_handler;
	sigaction(SIGINT, &signals, NULL);
	sigaction(SIGTERM, &signals, NULL);
	sigaction(SIGUSR1, &signals, NULL);
	sigaction(SIGUSR2, &signals, NULL);
	sigaction(SIGHUP, &signals, NULL);

	int rc = 0;

	struct timeval tv = {0};
    
    	while (!signal_exit) {
		fd_set rset;
		FD_ZERO(&rset);

		int rd, wr;

		FD_SET(input, &rset);
		FD_SET(fd, &rset);
	       
		LOG_TRACE("Polling I/O.");

		if (input_buffer_cursor || inverter_read_buffer_cursor) {
			tv.tv_sec = 0;
			tv.tv_usec = INVERTER_HID_BUFFER_SIZE * INVERTER_TIMEOUT_USEC;
		}

		rd = select(select_fds, &rset, NULL, NULL, input_buffer_cursor || inverter_read_buffer_cursor ? &tv : NULL);

		if (rd < 0) {
			if (errno == EINTR) {
				continue;
			}

			LOG_ERROR("Error waiting for input. %s (%d)", strerror(errno), errno);
			rc = 3;
			break;
		}

		if (!rd) {
			if (input_buffer_cursor) {
				LOG_ERROR("I/O timeout. Flushing partial command");
				input_buffer_cursor = 0;
			}

			if (inverter_read_buffer_cursor) {
				LOG_ERROR("I/O timeout. Flushing partial response");
				inverter_read_buffer_cursor = 0;
			}

			continue;
		}

		LOG_TRACE("I/O available.");

		if (FD_ISSET(input, &rset)) {
			LOG_TRACE("Reading from input.");
			rd = read_line(input, input_buffer, LINE_BUFFER_SIZE, &input_buffer_cursor, FLAG_ZERO_TERM);
			
			if (rd < 0) {
				LOG_ERROR("Error reading input. %s (%d)", strerror(errno), errno);
				rc = 6;
				break;
			}

			if (rd) {
				wr = inverter_write(fd, input_buffer);

				if (wr) {
					LOG_ERROR("Error writing to '%s'. %s (%d)", device, strerror(errno), errno);
					rc = 7;
					break;
				}
			}
		}

		if (FD_ISSET(fd, &rset)) {
			LOG_TRACE("Reading from device.");
			rd = read_line(fd, inverter_read_buffer, LINE_BUFFER_SIZE, &inverter_read_buffer_cursor, FLAG_READ_HID_BLOCK);
			
			if (rd < 0) {
				LOG_ERROR("Error reading from '%s'. %s (%d)", device, strerror(errno), errno);
				rc = 4;
				break;
			}

			if (rd) {
				LOG_TRACE("Read %u bytes", rd);

				char * recv = inverter_read(inverter_read_buffer, rd);

				if (recv) {
					rd = strlen(recv);
					recv[rd++] = separator;
					wr = write(output, recv, rd);

					if (wr != rd) {
						LOG_ERROR("Error writing output. %s (%d)", strerror(errno), errno);
						rc = 5;
						break;
					}
				}
			}
		}

	}

	inverter_close(fd);

	if (pty) {
		unlink(pty);
		close(input);
	}

	return rc;
}

