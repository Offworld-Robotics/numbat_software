#ifndef BLUETONGUE_H
#define BLUETONGUE_H

#include <ros/ros.h>
#include <string>
//#include <cstdint>
#include <vector>
#include <sensor_msgs/JointState.h>
#include "message.h"

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
        bool comm(void *message, int message_len);
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
        struct status update(double driveData[NUM_MSG]);
    
        bool reconnect(void);
        ros::NodeHandle nh;
};
#endif
