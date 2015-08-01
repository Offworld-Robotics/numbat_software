/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
#ifndef JOY_CONVERTER_H
#define JOY_CONVERTER_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
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

#define OPEN  0
#define STOP  1
#define CLOSE 2

//serial IO
#define TTY "/dev/ttyACM2"

class ClawControl {
    public:
        ClawControl();
        void run();
        
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        void sendMessage();
        ros::NodeHandle nh;
        ros::Publisher  velPublisher;
        ros::Subscriber joySubscriber;
        int topDrive, bottomDrive;
        int clawState;  
        
        //to keep track of button states. It is possible press could change it
        int cam0Button, cam1Button, cam2Button, cam3Button;
        
        //serial i0
        FILE * fd;
};

#endif
