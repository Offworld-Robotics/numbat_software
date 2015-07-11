/*
 * PathingController is the node for reading in (subscribing to) position and destination information
 * and adjusting (publishing) twist velocities to the arduino controller
 * Author: Simon Ireland for Bluesat OWR
 * Date: 11/07/2015
 */
 
 //#include "bluesat_owr_protobuf/Message1Relay.h"
#include "PathingController.h" 
#include <iostream>

#define TOPIC "/owr/pathing"
 
int main(int argc, char ** argv) {
    
    //init ros
    ros::init(argc, argv, "owr_pathing_node");
    
    PathingController p(TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

PathingController::PathingController(const std::string topic) {
    altitude = 0;
    latitude = 0;
    longitude = 0;
    pitch = 0;
    roll = 0;
    heading = 0;
    twistPublisher =  node.advertise<geometry_msgs::Twist>(topic,1000,true);
    positionSubscriber = node.subscribe("/gps/fix", 1000, &PathingController::receivePosMsg, this); // Position related data
    destinationSubscriber = node.subscriber("", 100, &PathingController::receiveDestMsg, this);
}

//TODO: Get the msg types for Destination and Position.
/* void PathingController::receivePosMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
   // altitude  = msg->altitude;
   // latitude  = msg->latitude;
   // longitude = msg->longitude;
   // sendMsg();
}

 
void PathingController::receiveDestMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
   // altitude  = msg->altitude;
   // latitude  = msg->latitude;
   // longitude = msg->longitude;
   // sendMsg();
}
*/

void PathingController::sendMsg() {
    //owr_messages::position msg;
    //msg.latitude = latitude;
    //msg.longitude = latitude;
    //msg.altitude = altitude;
    //msg.pitch = pitch;
    //msg.roll = roll;
    //msg.heading = heading;
    
    twistPublisher.publish(msg);
}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
