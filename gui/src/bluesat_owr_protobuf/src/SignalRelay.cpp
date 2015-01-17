/*
 * PBuff implementation
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 13/12/14
 */
 
 //protobuf changes variable names to lower case

#include "bluesat_owr_protobuf/SignalRelay.h"
 
#include <iostream>


void SignalRelay::reciveMsg(const MESSAGE_CLASS_ROS::ConstPtr& rosMsg){
    MESSAGE_CLASS pbuffMsg;
    ROS_INFO("Recived start");
    pbuffMsg.set_signalvalue(rosMsg->signalValue);
    ROS_INFO("Recived");
    pbuffMsg.SerializeToOstream(&std::cout);
    std::cout.flush();
}

MESSAGE_CLASS_ROS SignalRelay::doPbuffToROS(MESSAGE_CLASS pbuffMsg) {
    MESSAGE_CLASS_ROS rosMsg;
    ROS_INFO("Converting...");
    rosMsg.signalValue = pbuffMsg.signalvalue();
    return rosMsg;
}

SignalRelay::SignalRelay (std::string topic) :PBuffRelay(topic) {}
