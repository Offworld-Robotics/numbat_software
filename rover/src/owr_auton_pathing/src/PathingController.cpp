 <cmath>
#include <math.h>

#define PUBLISH_TOPIC "/owr/auto_pathing"
#define POS_TOPIC "/owr/position"
#define DEST_TOPIC "/owr/dest"
 
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
    
    geometry_msgs::Twist vel;
    
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
    	//Go straight

    } else if (( (currHeading +180) % 360) == destHeading){
    	//Go backwards

    } else {
    	angle = destHeading - currHeading;

    	if(angle > 0 && angle > (currHeading - 360)){
    		//turn left

    	} else {
    		//turn right

    	}
    }

    twistPublisher.publish(vel);
}

//main loop
void PathingController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}

