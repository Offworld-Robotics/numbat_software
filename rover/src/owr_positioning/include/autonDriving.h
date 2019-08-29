#ifdef AUTONOMOUS_DRIVING_H
#define AUTONOMOUS_DRIVING_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class PathingController {
	private:
		//setup stuff for nodes
		ros::Publisher pub;	//publish veloicty msg for rover 
		ros::Subscriber sub;	//read odometry/filtered for rover location
		ros::NodeHandle nh;		

		//functions
		void setWaypoint(double x, double y);
		geometry_msgs::Pose getRoverPose();
		geometry_msgs::Pose getCurrentWaypoint(int waypointNumber);
		geometry_msgs::Twist calculateVelocty(geometry_msgs::Pose roverLocation, geometry_msgs::Pose waypointLocation);

	public:
		PathingController();
		void startNode();
}

#endif //AUTONOMOUS_DRIVING_H
