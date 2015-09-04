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

#define PUBLISH_TOPIC "/owr/auton_twist"
#define POS_TOPIC "/owr/position"
#define POS_DODGE_TOPIC "/owr/position/dodge"
#define DEST_TOPIC "/owr/dest"
//#define EARTH_RADIUS 6371009
#define EQUATORIAL_RADIUS 63781370.0
#define POLAR_RADIUS 63567523.0
#define CHECKPOINT_DISTANCE 5
 


// Defines how fast the rover accelerates or starts turning, the range of output is -1 to 1, so currently 1/10th of range
#define INCREMENT 0.025
#define MAX_POW 1.0

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
    earthRadius = 6371009; // mean in meters, initial value but will be calculated throughout
    go = true;

    twistPublisher =  node.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1000,true);

    positionSubscriber = node.subscribe(POS_TOPIC, 10, &PathingController::receivePosMsg, this);
    dodgeSubscriber = node.subscribe(POS_DODGE_TOPIC, 10, &PathingController::receiveDodgeMsg, this);
    destinationSubscriber = node.subscribe(DEST_TOPIC, 10, &PathingController::receiveDestMsg, this);
}

// Get the msg types for Destination and Position.
void PathingController::receivePosMsg(const owr_messages::position &msg) {
   currLat = msg.latitude;
   currLong = msg.longitude;
   currHeading = msg.heading;
   sendMsg();
}

void PathingController::receiveDodgeMsg(const geometry_msgs::Twist::ConstPtr& velMsg) {
    if(velMsg->linear.x != 0) {
        go = false;
        vel.linear.x = velMsg->linear.x;
        vel.linear.y = velMsg->linear.y;
    } else {
        go = true;
    }
    sendMsg();
}

 
void PathingController::receiveDestMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> &msg) {
   destLat = msg->latitude;
   destLong = msg->longitude;
}


void PathingController::sendMsg() {
    if(!go) {
        twistPublisher.publish(vel);
    }
    //Calculate desired heading, first convert lat/longs into radians

    double lat1 = currLat * (M_PI / 180);
    double lat2 = destLat * (M_PI / 180);
    double long1 = currLong * (M_PI / 180);
    double long2 = destLong * (M_PI / 180);
    
    //Calculate Radius
    earthRadius = sqrt( ( pow((pow(EQUATORIAL_RADIUS, 2) * cos(lat1)), 2) + pow(pow(POLAR_RADIUS, 2) * sin(lat1), 2) ) / ( pow(EQUATORIAL_RADIUS * cos(lat1), 2) + pow(POLAR_RADIUS * sin(lat1), 2) ));


    // Check distance between currPosition and destination to check whether we are at destination
    double diffLat = lat2 - lat1;
    double diffLong = long2 - long1;
    double a = pow(sin(diffLat/2), 2) + cos(lat1)*cos(lat2)*pow(sin(diffLong/2), 2.0);
    double c = 2*atan2(sqrt(a), sqrt(1.0 - a));
    double distance = (earthRadius / 10) * c;

	ROS_INFO("distance %f earthRadius: %f destLat %f destLon %f\n", distance, earthRadius, destLat, destLong);

	if (distance < CHECKPOINT_DISTANCE || destLat == 0){
		// Within 10m of destination... if needed, can set up a counter in here to increment until we a point where we call for next destination.
		if(currPower){
		
			currPower -= INCREMENT * (currPower / fabs(currPower));
		}
        if(currLR){
        	//currLR -= INCREMENT * (currLR / fabs(currLR));
        }
		ROS_INFO("At destination! You are awesome!\n");
	} else {

		// Find the bearing from the lat and long values of position and destination: 'http://www.ig.utexas.edu/outreach/googleearth/latlong.html'
		double angle = atan2( (cos(lat2) * sin(long2 - long1)), ((sin(lat2) * cos(lat1)) - (sin(lat1) * cos(lat2) * cos(long2 - long1))));
		destHeading = fmod((angle * 180.0 / M_PI) + 360.0, 360.0);
		double reverseHeading = fmod((currHeading + 180.0), 360.0);
        #define HEADING_MARGIN 4
		// Work out the desired action to be taken
		if (currHeading > destHeading - HEADING_MARGIN && currHeading < destHeading + HEADING_MARGIN){
			//Go straight, decrement lr
			currPower += INCREMENT;
			currLR = 0.0;
		} else if (reverseHeading <= destHeading + HEADING_MARGIN &&
		    reverseHeading >= destHeading - HEADING_MARGIN){
			//Go backwards, decrement lr
			currPower -= INCREMENT;
			if(currLR != 0.0){
				currLR -= INCREMENT * (currLR / fabs(currLR));
			}
		} else {
			angle = destHeading - currHeading;

			if(fmod(angle+360.0, 360.0) > 180 && fmod(angle+360.0, 360.0) < 360){
				//turn left
				currPower += INCREMENT;
				currLR -= INCREMENT;

			} else {
				//turn right
				currPower += INCREMENT;
				currLR += INCREMENT;
			}
		}
	}
    if(currPower > MAX_POW) {
    	currPower = MAX_POW;
    } else if(currPower < -MAX_POW){
    	currPower = -MAX_POW;
    } else if (currPower < (0.9 * INCREMENT) && currPower > -(0.9 * INCREMENT)){
    	currPower = 0; //at least in soft testing, have found that the LR and pwr dont return back to 0 very well (end up 0.099...)
    } 

    if(currLR > 1.0){
    	currLR = 1.0;
    } else if(currLR < -1){
    	currLR = -1.0;
    } else if (currLR < (0.9 * INCREMENT) && currLR > -(0.9 * INCREMENT)){
    	currLR = 0; //at least in soft testing, have found that the LR and pwr dont return back to 0 very well (end up 0.099...)
    }

    ROS_INFO("pwr: %f, lr: %f \n", currPower, currLR);

    //Send twist message
    vel.linear.x = currPower;
	vel.linear.y = 0;
	//vel.linear.y = currLR;
	twistPublisher.publish(vel);

}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}

