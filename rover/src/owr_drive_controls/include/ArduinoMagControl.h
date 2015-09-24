
#ifndef ARDUINO_MAG_CONTROL_H
#define ARDUINO_MAG_CONTROL_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <owr_messages/heading.h>
#include <termios.h>
#include <stdio.h>
#include "buttons.h"

using namespace std; 

#define TOP_ACTUATOR 0
#define BOTTOM_ACTUATOR 1

#define FULL_EXTENSION 1700
//1700
#define FULL_RETRACTION 1200

#define DEFAULT_POS 1500

#define INCREMENT 10

//serial IO
#define TTY "/dev/ttyACM0"

class ArduinoMagControl {
    public:
        ArduinoMagControl();
        void run();
        
    private:
        ros::NodeHandle node;
        ros::Publisher  pub;
        double heading;
        
        
        
        
        //serial i0
        FILE * fd;
};

#endif
