/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef POS_CONTROL_H
#define POS_CONTROL_H

#include <ros/ros.h>
#include "owr_messages/position.h"
#include "owr_messages/heading.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>

class JoystickFilter {
    
    public:
        JoystickFilter(const std::string topic);
        void spin();
    protected:
        
        void sendMsg();
        ros::Publisher  publisher;   
        ros::Subscriber gpsSubscriber;  
        ros::Subscriber headingSubscriber;  

    private:
        //used to calculate the heading
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void armCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        
        //ros::Subscriber sub;
        ros::NodeHandle node;         // ros::NodeHandle nh;
        ros::Publisher  velPublisher;
        ros::Subscriber joySubscriber;
        ros::Subscriber armSubscriber;

        sensor_msgs::Joy msgsOut;
        
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
        


        
        //to keep track of button states. It is possible press could change it

   
};


#endif
