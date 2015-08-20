#ifndef BLUETONGUE_H
#define BLUETONGUE_H

#include <string>
//#include <cstdint>
#include <vector>

#define GPS_FLOAT_OFFSET 1000000

struct gpsData {
    uint16_t time;
    int32_t latitude; // lat * 10000 - to avoid floats
    int32_t longitude; // lon * 10000 - to avoid floats
    uint16_t numSatelites;
    int16_t altitude;
    uint16_t fixValid; //bool
} __attribute__((packed));
typedef struct gpsData GPSData;

struct magData {
    int16_t x, y, z;
    int16_t padding;
} __attribute__((packed));
typedef struct magData MagData;

struct imuData {
        int16_t gx, gy, gz;
        int16_t ax, ay, az;
} __attribute__((packed)); 
typedef struct imuData IMUData;

struct status {
    bool roverOk;
    double batteryVoltage;
    GPSData gpsData;
    MagData magData;
    IMUData imuData;
};
	
class Bluetongue {
	private:
		void comm(bool forBattery, void *message, int message_len, void *resp, 
				int resp_len);
		int port_fd;
        fd_set uart_set;
        struct timeval timeout;
	
	public:
		Bluetongue(const char* port);
		~Bluetongue();
		struct status update(double leftMotor, double rightMotor, int armTop,
                int armBottom, double armRotate);
};
#endif
