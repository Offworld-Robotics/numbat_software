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

#define PUBLISH_TOPIC "owr/auton_twist"
#define POS_TOPIC "/owr/position"
#define DEST_TOPIC "/owr/dest"
#define EARTH_RADIUS 6,371
#define CHECKPOINT_DISTANCE 0.005
 


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
   currLat = msg.latitude;
   currLong = msg.longitude;
   currHeading = msg.heading;
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
    
    // Check distance between currPosition and destination to check whether we are at destination
    double diffLat = lat2 − lat1;
    double diffLong = long2 − long1;
    double a = pow(sin(diffLat/2), 2) + cos(lat1)*cos(lat2)*pow(sin(diffLong/2), 2.0);
    double c = 2*atan2(sqrt(a), sqrt(1.0 − a));
    double distance = EARTH_RADIUS * c;

	if (distance < CHECKPOINT_DISTANCE){
		// Within 10m of destination... if needed, can set up a counter in here to increment until we a point where we call for next destination.
	    ROS_INFO("At destination! You are awesome! \n");
	} else {

    // Find the bearing from the lat and long values of position and destination: 'http://www.ig.utexas.edu/outreach/googleearth/latlong.html'
    double angle = atan2( (cos(lat2) * sin(long2 - long1)), ((sin(lat2) * cos(lat1)) - (sin(lat1) * cos(lat2) * cos(long2 - long1))));
    destHeading = fmod((angle * 180.0 / M_PI) + 360.0, 360.0);

    // Work out the desired action to be taken
    if (currHeading == destHeading){
    	//Go straight, decrement lr
        currPower += INCREMENT;
        if(currLR){
        	currLR -= 0.1 * (currLR / fabs(currLR));
        }
    } else if (fmod((currHeading + 180.0), 360.0) == destHeading){
    	//Go backwards, decrement lr
        currPower -= INCREMENT;
        if(currLR){
            currLR -= 0.1 * (currLR / fabs(currLR));
        }
    } else {
    	angle = destHeading - currHeading;

    	if(angle > 180 && angle < 360){
    		//turn left
            currPower += INCREMENT;
            currLR -= INCREMENT;

    	} else {
    		//turn right
            currPower += INCREMENT;
            currLR += INCREMENT;
    	}
    }

    if(currPower > 1){
    	currPower = 1;
    } else if(currPower < -1){
    	currPower = -1;
    } else if (currPower < INCREMENT && currPower > -INCREMENT){
    	currPower = 0; //at least in soft testing, have found that the LR and pwr dont return back to 0 very well (end up 0.099...)
    }

    if(currLR > 1){
    	currLR = 1;
    } else if(currLR < -1){
    	currLR = -1;
    } else if (currLR < INCREMENT && currLR > -INCREMENT){
    	currLR = 0; //at least in soft testing, have found that the LR and pwr dont return back to 0 very well (end up 0.099...)
    }

    ROS_INFO("pwr: %f, lr: %f \n", currPower, currLR);

    //Send twist message
    vel.linear.x = currPower;
	vel.linear.y = currLR;
	twistPublisher.publish(vel);
	}
}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
