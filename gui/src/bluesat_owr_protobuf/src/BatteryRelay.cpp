/*
 * PBuff implementation
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

#include "bluesat_owr_protobuf/BatteryRelay.h"
 
#include <iostream>
 
 


void BatteryRelay::reciveMsg(const MESSAGE_CLASS_ROS::ConstPtr& rosMsg){
    MESSAGE_CLASS pbuffMsg;
    ROS_INFO("Recived start");
    pbuffMsg.set_voltage(rosMsg->voltage);
    ROS_INFO("Recived");
    pbuffMsg.SerializeToOstream(&std::cout);
    std::cout.flush();
}

MESSAGE_CLASS_ROS BatteryRelay::doPbuffToROS(MESSAGE_CLASS pbuffMsg) {
    MESSAGE_CLASS_ROS rosMsg;
    ROS_INFO("Converting...");
    rosMsg.voltage = pbuffMsg.voltage();
    return rosMsg;
}

BatteryRelay::BatteryRelay (std::string topic) :PBuffRelay(topic) {}
