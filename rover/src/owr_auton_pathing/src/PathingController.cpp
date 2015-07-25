/*
 * PathingController is the node for reading in (subscribing to) position and destination information
 * and adjusting (publishing) twist velocities to the arduino controller
 * Author: Simon Ireland for Bluesat OWR
 * Date: 11/07/2015
 */
 
 //#include "bluesat_owr_protobuf/Message1Relay.h"
#include "PathingController.h" 
#include <iostream>

#define TOPIC "/owr/auto_pathing"
 
int main(int argc, char ** argv) {
    
    //init ros
    ros::init(argc, argv, "owr_pathing_node");
    
    PathingController p(TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

PathingController::PathingController(const std::string topic) {
    
    destLatitude = 0;
    destLongitude = 0;
    currLatitude = 0;
    currLongitude = 0;
    currHeading = 0;
    
    
    twistPublisher =  node.advertise<geometry_msgs::Twist>(topic,1000,true);
    positionSubscriber = node.subscribe("/owr/position", 1000, &PathingController::receivePosMsg, this); // Position related data
    destinationSubscriber = node.subscriber("", 100, &PathingController::receiveDestMsg, this);
}

//TODO: Get the msg types for Destination and Position.
/* void PathingController::receivePosMsg(const owr_messages::position & msg) {
   currLatitude = msg->latitude;
   currLongitude = msg->longitude;
   currHeading = msg->heading;
   sendMsg();
}

 
void PathingController::receiveDestMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
   destLatitude = msg->latitude;
   destLongitude = msg->longitude;
}
*/

void PathingController::sendMsg() {
    
    // TODO: calculate the correct twist message to send to the drive controls
    geometry_msgs::Twist vel;
    
    //TODO: Calculate desired heading. Requires calculating metres from lat/long
    // values, then working a bearing, considering possible quadrants of the angles.
    double latDist = destLatitide - currLatitude;
    double longDist = destLongitude - currLongitude;
    double destHeading = atan2(longDist, latDist);
    
    //TODO: Calculate whether we should be turning
    
    
    //TODO: Move forward if correct heading
    
    twistPublisher.publish(msg);
}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
