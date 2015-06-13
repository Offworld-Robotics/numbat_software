/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "PositionController.h" 
#include <iostream>

#define TOPIC "/owr/position"
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    PositionController p(TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

PositionController::PositionController(const std::string topic) {
    altitude = 0;
    latitude = 0;
    longitude = 0;
    pitch = 0;
    roll = 0;
    heading = 0;
    publisher =  node.advertise<owr_messages::position>(topic,1000,true);
    gpsSubscriber = node.subscribe("/gps/fix", 1000, &PositionController::receiveGPSMsg, this); // GPS related data
}

void PositionController::receiveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
    altitude  = msg->altitude;
    latitude  = msg->latitude;
    longitude = msg->longitude;
    sendMsg();
}

void PositionController::sendMsg() {
    owr_messages::position msg;
    msg.latitude = latitude;
    msg.longitude = latitude;
    msg.altitude = altitude;
    msg.pitch = pitch;
    msg.roll = roll;
    //msg.heading = heading;
    
    publisher.publish(msg);
}

//main loop
void PositionController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
