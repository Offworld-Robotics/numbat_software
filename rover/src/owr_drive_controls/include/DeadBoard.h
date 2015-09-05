#ifndef DeadBoard_H
#define DeadBoard_H

#include <string>
//#include <cstdint>
#include <vector>

#define GPS_FLOAT_OFFSET 1000000

//enable voltmeter code
#define VOLTMETER_ON 

struct dead_gpsData {
    uint16_t time;
    int32_t latitude; // lat * 10000 - to avoid floats
    int32_t longitude; // lon * 10000 - to avoid floats
    uint16_t numSatelites;
    int16_t altitude;
    uint16_t fixValid; //bool
} __attribute__((packed));
typedef struct dead_gpsData dead_GPSData;

struct dead_magData {
    int16_t x, y, z;
    int16_t padding;
} __attribute__((packed));
typedef struct dead_magData dead_MagData;

struct dead_imuData {
        int16_t gx, gy, gz;
        int16_t ax, ay, az;
} __attribute__((packed)); 
typedef struct dead_imuData dead_IMUData;

struct dead_status {
    bool roverOk;
    bool isConnected;
    double batteryVoltage;
    #ifdef VOLTMETER_ON
    double voltmeter;
    #endif
    dead_GPSData gpsData;
    dead_MagData magData;
    dead_IMUData imuData;
};
	
class DeadBoard {
	private:
		bool comm(bool forBattery, void *message, int message_len, void *resp, 
				int resp_len);
        bool connect();
        bool isConnected;
		int port_fd;
        std::string DeadBoard_port;
        fd_set uart_set;
        struct timeval timeout;
	
	public:
		DeadBoard(const char* port);
		~DeadBoard();
		struct dead_status update(double leftMotor, double rightMotor, 
                int armTop, int armBottom, double armRotate, 
                int clawRotate, int clawGrip, int cameraBottomRotate,
                int cameraBottomTilt, int cameraTopRotate, 
                int cameraTopTilt);
        bool reconnect(void);
};
#endif
