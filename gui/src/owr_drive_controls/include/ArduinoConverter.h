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
using namespace std; 

//joystick array indexes
#define BUTTON_A       0
#define BUTTON_B       1
#define BUTTON_X       2 
#define BUTTON_Y       3
#define BUTTON_LB      4
#define BUTTON_RB      5
#define BUTTON_BACK    6
#define BUTTON_START   7
#define BUTTON_POWER   8
#define BUTTON_STICK_L 9
#define BUTTON_STICK_R 10

//LR = Left Right, UD = Up Down
#define STICK_L_LR     0
#define STICK_L_UD     1
#define STICK_LT       2
#define STICK_R_LR     3
#define STICK_R_UD     4
#define STICK_RT       5
#define STICK_CH_LR    6
#define STICK_CH_UD    7


//and then the layout
//right stick for navigation
#define DRIVE_AXES_UD   STICK_R_UD
#define DRIVE_AXES_LR   STICK_R_LR

//a,b,x,y buttons for Camera feeds
#define CAM_FEED_0   BUTTON_A
#define CAM_FEED_1   BUTTON_B
#define CAM_FEED_2   BUTTON_X
#define CAM_FEED_3   BUTTON_Y

//serial IO
#define TTY "/dev/ttyACM1"

class ArduinoConverter {
    public:
        ArduinoConverter();
        void run();
        
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        ros::NodeHandle nh;
        ros::Publisher  velPublisher;
        ros::Subscriber joySubscriber;
        float leftDrive, rightDrive;
        
        //to keep track of button states. It is possible press could change it
        int cam0Button, cam1Button, cam2Button, cam3Button;
        
        //serial i0
        FILE * fd;
};

#endif
