/*
 * LocalPlanenr is the node for reading in (subscribing to) the astar Path
 * and publishing twist velocities to cmdVelToJoints to make the rover
 * naviagate autonomously. This node will also allow for local object detection
 * via the lidar detection system
 *
 * Author: Simon Ireland
 * Date: 5/8/2016
 *
 * NODE NAME: 
 * topics used:
 */
 
#include "LocalPlanner.hpp"


int main (int argc, char *argv[]) {
    
    ros::init(argc, argv, "owr_local_planner");
    LocalPlanner planner;
    
    planner.run();
    
    return 0;
}


void LocalPlanner::run(){
    while(ros::ok()){
        
        //if no message has yet been received, wait, otherwise, use navPath to determine a twist
        if(!received){
            ros::spinOnce();
        } else {
            
            // Message to be sent, defining the new direction of travel
            geometry_msgs::Twist vel_msg;
            
            double navX, navY;
            double roll, pitch, yaw;
            double driveX, driveY;
            
            
            // Query the listener for a specific transform, that being:
            // From: arg0 = "map" 
            // To: arg1 = "base_link" 
            // At the time: arg2 = navPath->header.stamp
            // The object we put the resulting transform: arg3 = currPosition
            tfListener.lookupTransform("map","base_link", navPath->header.stamp, currPosition);
            
            
            //TODO: Insert local Lidar obstacle handling
            
            
            // Create a vector from currPosition to first <Pose> within navPath
            tf::Vector3 desiredVector( navPath.poses[0].position.x - currPosition.transform.translation.x,
                                       navPath.poses[0].position.y - currPosition.transform.translation.y, 
                                       0);
            
            // Find the orientation of the rover
            tf::Matrix3x3 m(currPosition.transform.rotation).getRPY( roll, pitch, yaw);
            
            // Calculate a heading vector from the yaw (which is in radians)
            tf::Vector3 headingVector( navX = cos(yaw), navY = sin(yaw), 0);
            
            // Take the cross-product of the desired and heading vectors to determine turning direction
            tf::Vector3 cross = desiredVector.cross(headingVector);
            
            // The result of the cross product will be a vector normal to the 2 vectors
            if(cross.z == 0.0f) {
                driveX = 0.7;
                driveY = 0;
            } else if(cross.z > 0 || cross.z == -0.0f) {
                driveX = 0.7;
                driveY = 1;
            } else {
                driveX = 0.7;
                driveY = -1;
            }
        }
    }
}


LocalPlanner::LocalPlanner() {
    
    // Setup subscriber to astar path
    pathSubscriber = nh.subscribe<nav_msgs::Path>(PATH_TOPIC, 1, &LocalPlanner::pathCallback, this);
    
    //Subscribe to lidar obstacle detection
    //lidarSubscriber = nh.subscribe<INSERT_MSG>(LIDAR_TOPIC, 1, &LocalPlanner::lidarCallback, this);
    
    //Subscribe to current orientation of the rover
    //orientSubscriber = nh.subscribe<INSERT_MSG>(ORIENT_TOPIC, 1, &LocalPlanner::orientCallback, this);
    
    //Setup twist publisher to cmdVelToJoints
    twistPublisher = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, 1, true);
    
    received = FALSE;
}


LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& pathIn){
    navPath = pathIn;
    received = TRUE;
}

LocalPlanner::lidarCallback(){
    //TODO: Lidar handling
}
