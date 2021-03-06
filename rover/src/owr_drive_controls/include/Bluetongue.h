#ifndef BLUETONGUE_H
#define BLUETONGUE_H

#include <ros/ros.h>
#include <string>
//#include <cstdint>
#include <vector>
#include <sensor_msgs/JointState.h>

#define GPS_FLOAT_OFFSET 1000000

#define NUM_JOINTS 16
#define LIDAR_JOINT 0
#define LEFT_MOT_JOINT 1
#define RIGHT_MOT_JOINT 2
#define ARM_TOP_JOINT 3
#define ARM_BOT_JOINT 4
#define ARM_ROT_JOINT 5
#define CLAW_ROT_JOINT 6
#define CLAW_GRIP_JOINT 7
#define CAM_BOT_ROTATE_JOINT 8
#define CAM_BOT_TILT_JOINT 9
#define CAM_TOP_ROT_JOINT 10
#define CAM_TOP_TILT_JOINT 11
#define EXTRA_1 12
#define EXTRA_2 13
#define EXTRA_3 14
#define EXTRA_4 15


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
    double swerveLeft; // Swerve Positions from potentiometers
    double swerveRight;
    
    double armUpper;
    double armLower;
    
    double pot0; // TODO: implement and rename when being used.
    double pot1;
    double pot2;
    double pot3;

    double clawActual;
    double clawEffort;
};
    
class Bluetongue {
    private:
        bool comm(bool forBattery, void *message, int message_len, void *resp, 
                int resp_len);
        bool connect();
        void publish_joint(std::string name, double position, double velocity, double effort, int jointNo);
        
        bool isConnected;
        int port_fd;
        std::string bluetongue_port;
        fd_set uart_set;
        struct timeval timeout;
        double timeSeq;
        
        
        sensor_msgs::JointState jointMsg;
    
    public:
        Bluetongue(const char* port);
        ~Bluetongue();
        struct status update(double leftFMotor, double rightFMotor, 
                double leftBMotor, double rightBMotor, double leftFSwerve, double rightFSwerve,
                int armTop, int armBottom, double armRotate, 
                int clawRotate, int clawGrip, int cameraBottomRotate,
                int cameraBottomTilt, int cameraTopRotate, 
                int cameraTopTilt, int lidarTilt);
//         void tf_lidar(int16_t pwm);
        bool reconnect(void);
        ros::Publisher lidarTFPublisher;
        ros::NodeHandle nh;
};
#endif
