/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 8/07/15
 */
#ifndef BOARD_CONTROL_H
#define BOARD_CONTROL_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <owr_messages/board.h>
#include <termios.h>
#include <stdio.h>
#include "Bluetongue.h"

using namespace std; 


//serial IO
#define TTY "/dev/ttyACM0"

//the minimum number of satelites required to make the fix valid
#define MIN_SATELITES 3

class BoardControl {
    public:
        BoardControl();
        void run();
        
    private:
        void controllerCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        void velCallback(const geometry_msgs::Twist::ConstPtr& vel);

        void publishGPS(GPSData gps);
        void publishMag(MagData mag);
        void publishIMU(IMUData imu);
        void publishBattery(double batteryVoltage);
        void publishVoltmeter(double voltage);
        //void sendMessage(float lf, float lm, float lb, float rf, float rm, float rb);
        ros::NodeHandle nh;
        ros::Publisher  velPublisher;

	    ros::Subscriber velSubscriber;
        ros::Publisher gpsPublisher;
        ros::Publisher magPublisher;
        ros::Publisher gyroPublisher;
        ros::Publisher accPublisher;
        ros::Publisher battVoltPublisher;
        ros::Publisher voltmeterPublisher;
        ros::Publisher boardStatusPublisher;
        ros::Subscriber joySubscriber;
        ros::Subscriber armSubscriber;
        float leftDrive, rightDrive;
        //arm top, bottom
        int armTop, armBottom;
        float armRotate;
        int armIncRate;
        int cameraBottomRotate, cameraBottomTilt, cameraTopRotate,
            cameraTopTilt;
        int cameraBottomRotateIncRate, cameraBottomTiltIncRate, 
            cameraTopRotateIncRate, cameraTopTiltIncRate;
        //gps sequence number
        int gpsSequenceNum;
        
        //claw stuff
        int clawState;
        int rotateState;
        int clawRotate; // pwm
        int clawGrip;
        int rotState; // On-Off
        //int clawState;
        
        //to keep track of button states. It is possible press could change it
        int cam0Button, cam1Button, cam2Button, cam3Button;
        
        //serial i0
        FILE * fd;
};

#endif
