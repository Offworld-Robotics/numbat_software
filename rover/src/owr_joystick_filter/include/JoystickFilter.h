/*
 * Filters the Joysticks 
 *  ie takes input from gamepad and sends appropriate control messages to 
 *  rover systems
 * Original Author: Sam S
 * Editors: Harry J.E Day, Sean Thompson
 * ROS_NODE:owr_joystick_filter
 */
 
#ifndef JOYSTICK_FILTER_H
#define JOYSTICK_FILTER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include "owr_messages/position.h"
#include "owr_messages/heading.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

class JoystickFilter {
    
    public:
        JoystickFilter(const std::string topic);
        void spin();
    protected:
        
        void sendMsg();
        ros::Publisher  publisher;   //nothing should go through this 
        ros::Subscriber gpsSubscriber;  
        ros::Subscriber headingSubscriber;  

    private:
        //used to calculate the heading
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void armCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        
        //ros::Subscriber sub;
        ros::NodeHandle node;         // ros::NodeHandle nh;
        ros::Publisher  velPublisher; // for the wheels
        
        // Publishers for the Arm
        ros::Publisher armUpperActPub;
	ros::Publisher armLowerActPub;
	ros::Publisher armBaseRotatePub;
	ros::Publisher clawRotateRub;
	ros::Publisher clawGripPub;
        
        ros::Publisher lidarModePublisher;
        ros::Publisher lidarPosPublisher;
	
	
        ros::Subscriber joySubscriber;
        ros::Subscriber armSubscriber;
        
        double gimbalRate;

        std::string topic;

        
        //variables we use to store calculated message data
        //these correspond to to the message variables owr_messages/position
        //Documented here: https://bluesat.atlassian.net/wiki/pages/viewpage.action?pageId=1770122
        double latitude;
        double longitude;
        double altitude;
        double pitch;
        double roll;
        double heading;
        std::list<double> latitudes;
        std::list<double> longitudes;
        int cam0Button, cam1Button, cam2Button, cam3Button;
        std_msgs::Int16 lidarModeMsg;
        std_msgs::Float64 lidarPos;

        
        // working data for thumbsticks calculated on each input callback
        // LEFT Thumb Stick
        Eigen::Vector2d rawLStick;
        double rawMagLStick;
        double deadZoneCorrectedMagL;
        double deadzoneRescaledLStickMag;
        Eigen::Vector2d rescaledLStick;
        // RIGHT Thumb Stick
        Eigen::Vector2d rawRStick;
        double rawMagRStick;
        double deadZoneCorrectedMagR;
        double deadzoneRescaledRStickMag;
        Eigen::Vector2d rescaledRStick;
};

#endif //JOYSTICK_FILTER_H
