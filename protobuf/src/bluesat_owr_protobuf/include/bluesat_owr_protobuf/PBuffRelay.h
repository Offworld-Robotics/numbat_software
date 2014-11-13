/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef PBuffRelay_H
#define PBuffRelay_H

#define MESSAGE_CLASS bluesat_owr_protobuf_proto::message1
#define MESSAGE_CLASS_ROS bluesat_owr_protobuf::message1_ros

#define TOPIC "/owr_protobuf/" "MESSAGE_CLASS"

#include "message1.pb.h"
#include "bluesat_owr_protobuf/message1_ros.h"
#include <ros/ros.h>

class PBuffRelay {
    
    public:
        PBuffRelay();
        void spin();
        
    private:
        ros::Subscriber sub;
        MESSAGE_CLASS testMessage;
        ros::NodeHandle node;
};


#endif
