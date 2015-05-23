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
        PositionController(std::string topic);
        void spin();
    protected:
        
        void reciveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) ;
        ros::Publisher  publisher;   
        ros::Subscriber subscriber;  
    
    private:
        ros::Subscriber sub;
        ros::NodeHandle node;
        std::string topic;
   
};


#endif
