/*
 * PBuff implementation
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

#include "bluesat_owr_protobuf/Message1Relay.h"
 
#include <iostream>
 
 


void Message1Relay::reciveMsg(const MESSAGE_CLASS_ROS::ConstPtr& rosMsg){
    MESSAGE_CLASS pbuffMsg;
    ROS_INFO("Recived start");
    pbuffMsg.set_x(rosMsg->x);
    pbuffMsg.set_y(rosMsg->y);
    pbuffMsg.set_z(rosMsg->z);
    ROS_INFO("Recived");
    pbuffMsg.SerializeToOstream(&std::cout);
    std::cout.flush();
}

MESSAGE_CLASS_ROS Message1Relay::doPbuffToROS(MESSAGE_CLASS pbuffMsg) {
    MESSAGE_CLASS_ROS rosMsg;
    ROS_INFO("Converting...");
    rosMsg.x = pbuffMsg.x();
    rosMsg.y = pbuffMsg.y();
    rosMsg.z = pbuffMsg.z();
    return rosMsg;
}

Message1Relay::Message1Relay (std::string topic) :PBuffRelay(topic) {}
