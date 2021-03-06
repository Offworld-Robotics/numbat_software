/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef POS_CONTROL_H
#define POS_CONTROL_H

#include <ros/ros.h>
#include "owr_messages/heading.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

class MagnetConverter {
    
    public:
        MagnetConverter(const std::string topic);
        void spin();
    protected:
        
        void receiveMsg(const boost::shared_ptr<geometry_msgs::Vector3Stamped const> & msg);
        ros::Publisher  publisher, pubDebug;   
        ros::Subscriber subscriber;  
    
    private:
        geometry_msgs::Quaternion hamiltonProduct(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);
        ros::NodeHandle node;
        std::string topic;
        
        
   
};


#endif
