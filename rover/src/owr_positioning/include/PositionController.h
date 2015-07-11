/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef POS_CONTROL_H
#define POS_CONTROL_H

#include <ros/ros.h>
#include "owr_messages/position.h"
#include <sensor_msgs/NavSatFix.h>

class PositionController {
    
    public:
        PositionController(const std::string topic);
        void spin();
    protected:
        
        void receiveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) ;
        void sendMsg();
        ros::Publisher  publisher;   
        ros::Subscriber gpsSubscriber;  
    
    private:
        //used to calcualte the heading
        void updateHeading();
        
        ros::Subscriber sub;
        ros::NodeHandle node;
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
        
   
};


#endif
