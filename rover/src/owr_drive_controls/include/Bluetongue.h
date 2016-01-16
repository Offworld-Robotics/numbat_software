#ifndef BLUETONGUE_H
#define BLUETONGUE_H

#include <ros/ros.h>
#include <string>
//#include <cstdint>
#include <vector>

#define GPS_FLOAT_OFFSET 1000000

#define LIDAR_HORIZ 1330
#define DEG_PER_PWM 0.12328767123

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
    bool isConnected;
    double batteryVoltage;
    #ifdef VOLTMETER_ON
    double voltmeter;
    #endif
    GPSData gpsData;
    MagData magData;
    IMUData imuData;
};
	
class Bluetongue {
	private:
		bool comm(bool forBattery, void *message, int message_len, void *resp, 
				int resp_len);
        bool connect();
        bool isConnected;
		int port_fd;
        std::string bluetongue_port;
        fd_set uart_set;
        struct timeval timeout;
        
        //ros::NodeHandle nh;
        //ros::Publisher lidarTFPublisher;
	
	public:
		Bluetongue(const char* port);
		~Bluetongue();
		struct status update(double leftMotor, double rightMotor, 
                int armTop, int armBottom, double armRotate, 
                int clawRotate, int clawGrip, int cameraBottomRotate,
                int cameraBottomTilt, int cameraTopRotate, 
                int cameraTopTilt, int lidarTilt);
        //void tf_lidar(int16_t pwm);
        bool reconnect(void);
};
#endif
