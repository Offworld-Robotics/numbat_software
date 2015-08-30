#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "Bluetongue.h"
#include <ros/ros.h>

using namespace std;

#define MESSAGE_MAGIC 0x55AA
#define DODGY_USB_CONNECTION 100
// All types are multiples of 16 bits, due to how XC16 optimises for memory access
// All structs are multiples of 32 bits, so this works on x86
struct toControlMsg {
    uint16_t magic;
    int16_t lSpeed;
    int16_t rSpeed;
    int16_t armRotate;
    int16_t armTop;
    int16_t armBottom;
    int16_t clawRotate;
    int16_t clawGrip;
    int16_t cameraBottomRotate;
    int16_t cameraBottomTilt;
    int16_t cameraTopRotate;
    int16_t cameraTopTilt;
} __attribute__((packed));

struct toNUCMsg {
    uint16_t magic;
    uint16_t vbat;
    GPSData gpsData;
    MagData magData;
    IMUData imuData;
} __attribute__((packed));

bool Bluetongue::reconnect(void) {
    if (access(bluetongue_port.c_str(), W_OK | R_OK) != -2) {
        close(port_fd);
    }
    isConnected = connect();
    return isConnected;
}

bool Bluetongue::connect() {
    port_fd = open(bluetongue_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (port_fd == -1) {
		ROS_ERROR("Error in open uart port");
        return false;
	} else {
		ROS_DEBUG("Opened uart port");
	}
    // Set up stuff for select so we can timeout on reads
    FD_ZERO(&uart_set); /* clear the set */
    FD_SET(port_fd, &uart_set); /* add our file descriptor to the set */
	timeout.tv_sec = 1;
    timeout.tv_usec = 0;

	// Set parameters
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Error Handling 
	if (tcgetattr(Bluetongue::port_fd, &tty) != 0) {
		ROS_ERROR("Error %d from tcgetattr: %s", errno, strerror(errno));
        close(port_fd);
        return false;
	}
	ROS_DEBUG("Setting up uart");

	// Set Baud Rate 
	cfsetospeed(&tty, (speed_t)B19200);
	cfsetispeed(&tty, (speed_t)B19200);

	// Setting other Port Stuff 
	tty.c_cflag &= ~PARENB; // Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	tty.c_cflag &= ~CRTSCTS; // no flow control
	tty.c_cc[VMIN] = 1; // read doesn't block
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
	tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
	ROS_DEBUG("About to make raw");

	// Make raw 
	cfmakeraw(&tty);

	// Flush Port, then applies attributes
	tcflush(port_fd, TCIFLUSH);
	if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
		ROS_ERROR("Error %d from tcsetattr: %s" ,errno, strerror(errno));
        close(port_fd);
        return false;
	}
    return true;
}

Bluetongue::Bluetongue(const char* port) {
	// Open serial port
    bluetongue_port = port;
    isConnected = connect();	
    ROS_INFO("Finished initalizing bluetongue");
}

Bluetongue::~Bluetongue(void) {
	close(Bluetongue::port_fd);
}

bool Bluetongue::comm(bool forBattery, void *message, int message_len, 
    void *resp, int resp_len) {
	ROS_DEBUG("Writing message: ");
	for (int i = 0; i < message_len; i++) {
		ROS_DEBUG("%d: %02x\n", i, *((char *) message + i));
	}
	int written = 0;
    timeout.tv_sec = 0;
    timeout.tv_usec = 400000;
    int empty_writes = 0;
	do {
		int write_amount = write(port_fd, (int8_t*)message + written, message_len - written);
        if (write_amount == -1) {
            ROS_ERROR("USB write error");
            return false;
        }
        written += write_amount;
        if (write_amount == 0) ++empty_writes;
        if (empty_writes > DODGY_USB_CONNECTION) {
            ROS_ERROR("Dodgy usb connection detected");
            return false;
        }
	} while (written < message_len);
	ROS_DEBUG("Written packet, expecting to read %d", resp_len);
	tcflush(port_fd, TCIOFLUSH); 
	int empty_reads = 0;
    int readCount = 0;
	do {
        int rv = select(port_fd + 1, &uart_set, NULL, NULL, &timeout);
        if(rv == -1) {
            ROS_ERROR("select"); /* an error accured */
            return false;
        } else if(rv == 0) {
            ROS_ERROR("timeout"); /* a timeout occured */
            return false; // This might not require a full reset...
        } else {
		  int read_amount = read(port_fd, (int8_t*)resp + readCount, resp_len - readCount);
          readCount += read_amount;
          if (read_amount == 0) ++empty_reads;
          if (empty_reads > DODGY_USB_CONNECTION) {
              ROS_ERROR("Dodgy usb connection detected");
              return false;
          }
		}
        ROS_DEBUG("reading... %d", readCount);
	} while (readCount < resp_len);
	ROS_DEBUG("Read packet");
    return true;
}

struct status Bluetongue::update(double leftMotor, double rightMotor, int armTop, 
    int armBottom, double armRotate, int clawRotate, int clawGrip,
    int cameraBottomRotate, int cameraBottomTilt, int cameraTopRotate,
    int cameraTopTilt) {
    struct status stat;
    if (!isConnected) {
        stat.isConnected = false;
        stat.roverOk = false;
        return stat;
    }
	struct toControlMsg mesg;
	struct toNUCMsg resp;
	mesg.magic = MESSAGE_MAGIC;
	mesg.lSpeed = (leftMotor * 500) + 1500; // Scale to 16bit int
	mesg.rSpeed = (rightMotor * 500) + 1500;
    mesg.armRotate = (armRotate * 500) + 1500;
    mesg.armTop = armTop;
    mesg.armBottom = armBottom;
    mesg.clawRotate = clawRotate;
    mesg.clawGrip = clawGrip;
    mesg.cameraBottomRotate = cameraBottomRotate;
    mesg.cameraBottomTilt = cameraBottomTilt;
    mesg.cameraTopRotate = cameraTopRotate;
    mesg.cameraTopTilt = cameraTopTilt;
    ROS_INFO("rotate %d grip %d", mesg.clawRotate, mesg.clawGrip);
	ROS_INFO("Speeds %d %d", mesg.lSpeed, mesg.rSpeed);
	ROS_INFO("Writing %d bytes.", (int) sizeof(struct toControlMsg));
	ROS_INFO("Arm top %d bottom %d rotate %d", mesg.armTop, 
            mesg.armBottom, mesg.armRotate);
    ROS_INFO("Camera br %d bt %d tr %d tt %d", cameraBottomRotate,
            cameraBottomTilt, cameraTopRotate, cameraTopTilt);
	isConnected = comm(false, &mesg, sizeof(struct toControlMsg), &resp, 
            sizeof(struct toNUCMsg));
	
    if (!isConnected) {
        stat.isConnected = false;
        stat.roverOk = false;
        return stat;
    } else {
        stat.isConnected = true;
    }
    
    if (resp.magic != MESSAGE_MAGIC) {
		ROS_INFO("Update Bluetongue had a error");
        stat.roverOk = false;    
        return stat;
	} else {
        stat.roverOk = true;
    }
    stat.batteryVoltage = ((resp.vbat / 1024.0) * 3.3) * 5.7;
    stat.gpsData = resp.gpsData;
    stat.magData = resp.magData;
    stat.imuData = resp.imuData;
    return stat;
}
