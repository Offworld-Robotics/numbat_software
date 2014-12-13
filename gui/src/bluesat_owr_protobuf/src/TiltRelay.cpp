/*
 * PBuff implementation
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 13/12/14
 */
 

#include "bluesat_owr_protobuf/TiltRelay.h"
 
#include <iostream>


void TiltRelay::reciveMsg(const MESSAGE_CLASS_ROS::ConstPtr& rosMsg){
    MESSAGE_CLASS pbuffMsg;
    ROS_INFO("Recived start");
    pbuffMsg.set_tiltx(rosMsg->tiltx);
    pbuffMsg.set_tilty(rosMsg->tilty);  
    ROS_INFO("Recived");
    pbuffMsg.SerializeToOstream(&std::cout);
    std::cout.flush();
}

MESSAGE_CLASS_ROS TiltRelay::doPbuffToROS(MESSAGE_CLASS pbuffMsg) {
    MESSAGE_CLASS_ROS rosMsg;
    ROS_INFO("Converting...");
    rosMsg.tiltx = pbuffMsg.tiltx();
    rosMsg.tilty = pbuffMsg.tilty();
    return rosMsg;
}

TiltRelay::TiltRelay (std::string topic) :PBuffRelay(topic) {}
