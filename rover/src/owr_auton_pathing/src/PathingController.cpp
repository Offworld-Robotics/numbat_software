/*
 * PathingController is the node for reading in (subscribing to) position and destination information
 * and adjusting (publishing) twist velocities to the arduino controller
 * Author: Simon Ireland for Bluesat OWR
 * Date: 11/07/2015
 *
 * NODE NAME: 'owr_auton_pathing'
 * topics used: see definitions
 */
 
#include "PathingController.h" 
#include <iostream>
#include <cmath>
#include <math.h>

#define PUBLISH_TOPIC "owr/control/drive"
#define POS_TOPIC "/owr/position"
#define DEST_TOPIC "/owr/dest"
 
// Defines how fast the rover accelerates or starts turning, the range of output is -1 to 1, so currently 1/10th of range
#define INCREMENT 0.1

int main(int argc, char ** argv) {
    
    //init ros
    ros::init(argc, argv, "owr_auton_pathing");
    
    PathingController p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

PathingController::PathingController( void) {
    
    destLat = 0;
    destLong = 0;
    currLat = 0;
    currLong = 0;
    destHeading = 0;
    currHeading = 0;
    currPower = 0;
    currLR = 0;
	vel.linear.x = 0;
	vel.linear.y = 0;
    
    twistPublisher =  node.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1000,true);

    positionSubscriber = node.subscribe(POS_TOPIC, 1000, &PathingController::receivePosMsg, this);
    destinationSubscriber = node.subscribe(DEST_TOPIC, 100, &PathingController::receiveDestMsg, this);
}

//TODO: Get the msg types for Destination and Position.
void PathingController::receivePosMsg(const owr_messages::position &msg) {
   currLat = msg->latitude;
   currLong = msg->longitude;
   currHeading = msg->heading;
   sendMsg();
}

 
void PathingController::receiveDestMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> &msg) {
   destLat = msg->latitude;
   destLong = msg->longitude;
}


void PathingController::sendMsg() {

    //Calculate desired heading, first convert lat/longs into radians

    double lat1 = currLat * (M_PI / 180);
    double lat2 = destLat * (M_PI / 180);
    double long1 = currLong * (M_PI / 180);
    double long2 = destLong * (M_PI / 180);
    
    // Find the bearing from the lat and long values of position and destination: 'http://www.ig.utexas.edu/outreach/googleearth/latlong.html'
    double angle = atan2( cos(destLat) * sin(destLong - currLong), sin(destLat) * cos(currLat) - sin(currLat) * cos(destLat) * cos(destLong - currLong));
    destHeading = ( (angle * 180 / M_PI) + 360) % 360;

    // Work out the desired action to be taken
    if (currHeading == destHeading){
    	//Go straight, decrement lr
        currPower += INCREMENT;
        currLR -= 0.1*(currLR / abs(currLR));

    } else if (( (currHeading +180) % 360) == destHeading){
    	//Go backwards, decrement lr
        currPower -= INCREMENT;
        currLR -= INCREMENT * (currLR / abs(currLR));

    } else {
    	angle = destHeading - currHeading;

    	if(angle > 0 && angle > (currHeading - 360)){
    		//turn left
            currLR -= INCREMENT;

    	} else {
    		//turn right
            currLR += INCREMENT;
    	}
    }

    if(currPower > 1){
    	currPower = 1;
    } else if(currPower < -1){
    	currPower = -1;
    }

    if(currLR > 1){
    	currLR = 1;
    } else if(currLR < -1){
    	currLR = -1;
    }

    //Send twist message
    vel.linear.x = currPower;
	vel.linear.y = currLR;
	twistPublisher.publish(vel);
}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
