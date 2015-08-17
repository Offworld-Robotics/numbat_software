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


//serial IO
#define TTY "/dev/ttyACM1"

class ArduinoConverter {
    public:
        ArduinoConverter();
        void run();
        
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        //void sendMessage(float lf, float lm, float lb, float rf, float rm, float rb);
        ros::NodeHandle nh;
        ros::Publisher  velPublisher;
        ros::Subscriber joySubscriber;
        float leftDrive, rightDrive;
        float lfDrive, lmDrive, lbDrive, rfDrive, rmDrive, rbDrive;
        
        //to keep track of button states. It is possible press could change it
        int cam0Button, cam1Button, cam2Button, cam3Button;
        
        //serial i0
        FILE * fd;
};

#endif
