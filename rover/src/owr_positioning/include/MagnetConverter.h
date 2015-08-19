/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef POS_CONTROL_H
#define POS_CONTROL_H

#include <ros/ros.h>
#include "owr_messages/position.h"
#include <sensor_msgs/Imu.h>

class MagnetConverter {
    
    public:
        MagnetConverter(const std::string topic);
        void spin();
    protected:
        
        void receiveMsg(const boost::shared_ptr<geometry_msgs::Vector3 const> & msg) ;
        ros::Publisher  publisher;   
        ros::Subscriber subscriber;  
    
    private
        
        ros::NodeHandle node;
        std::string topic;
        
        
   
};


#endif
