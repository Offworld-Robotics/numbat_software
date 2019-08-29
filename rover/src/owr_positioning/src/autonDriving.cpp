// original author: Nikola Medimurac
// date started: 28 / 8 / 2019

// new path following method to try for ERC2019
// method will involve the following
// 1. input a series of waypoints the rover should drive to in order
// 2. read rover position from odometry/filtered topic
// 3. find angle the rover is facing and angle it needs to go to waypoint
// 4. use PID controller to adjust rover so that it drives toward waypoint

#include "autonDriving.h"


//main function to start up the node
int main(int argc char *argv[]) {
	ros::init(argc, argv, "path_controller");
	PathingController p
	
	while(ros::ok()) {
		ros::spinOnce();
	}
}

//contructor, initialise node stuff and setup waypoints
PathingController::PathingController(){
	//setup subscriber and publisher
	sub = nh.subscribe("/odometry/filtered", 1, &PathingController::callback, this);
	pub = nh.advertise("topic_name", 1);
}
//functions
void setWaypoint(double x, double y);
geometry_msgs::Pose getRoverPose();
geometry_msgs::Pose getCurrentWaypoint(int waypointNumber);
geometry_msgs::Twist calculateVelocty(geometry_msgs::Pose roverLocation, geometry_msgs::Pose waypointLocation);

