/*
 * Converts Twist message input (subscribe) to arduino output, etc
 * Author: Simon Ireland for BlueSat OWR
 * Start: 20/06/15
 *
 */

//#ifndef JOY_CONVERTER_H
//#define JOY_CONVERTER_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <stdio.h>
#include "buttons.h"

using namespace std;

//serial IO
#define TTY "/dev/ttyACM1"

class RosArduinoMotor {
    public:
		RosArduinoMotor();
        void run();

    private:
        void sendMessage(float lf, float lm, float lb, float rf, float rm, float rb);
        void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
        ros::NodeHandle nh;
        ros::Subscriber velSubscriber;
        float leftDrive, rightDrive;
        float lfDrive, lmDrive, lbDrive, rfDrive, rmDrive, rbDrive;

        //serial i0
        FILE * fd;
};

//#endif
