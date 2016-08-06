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
            
            double navX = 0.0;
            double navY = 0.0;
            double roll, pitch, yaw;
            double driveX, driveY;
            double distance;
            
            
            // Query the listener for a specific transform, that being:
            // From: arg0 = "map" 
            // To: arg1 = "base_link" 
            // At the time: arg2 = navPath->header.stamp
            // The object we put the resulting transform: arg3 = currPosition
            tfListener.lookupTransform("map","base_link", navPath.header.stamp, currPosition);
            
            
            //TODO: Insert local Lidar obstacle handling
            
            
            // Create a vector from currPosition to first <Pose> within navPath
            tf::Vector3 desiredVector( navPath.poses[count].pose.position.x - currPosition.getOrigin().x(),
                                       navPath.poses[count].pose.position.y - currPosition.getOrigin().y(), 
                                       0);
            
            // Find the orientation of the rover
            tf::Matrix3x3 m(currPosition.getRotation());
            
            m.getRPY( roll, pitch, yaw);
            
            // Calculate a heading vector from the yaw (which is in radians)
            tf::Vector3 headingVector( navX = cos(yaw), navY = sin(yaw), 0);
            
            // Take the cross-product of the desired and heading vectors to determine turning direction
            tf::Vector3 cross = desiredVector.cross(headingVector);
            
            
            distance = desiredVector.length();
            
            //Normalise vector
            //TODO: use this to define turn speed
            cross.normalize();
            
            if(distance > 0.8){
                
                // The result of the cross product will be a vector normal to the 2 vectors
                //TODO: ensure straight forward has a dead-zone
                if(cross.getZ() == 0.0f) {
                    driveX = MAX_SPEED;
                    driveY = 0;
                } else if(cross.getZ() > 0 || cross.getZ() == -0.0f) {
                    driveX = MAX_SPEED;
                    driveY = 1;
                } else {
                    driveX = MAX_SPEED;
                    driveY = -1;
                }
                
                if( abs( cross.getZ( ) ) <= 0.3){
                    driveY = 2 * driveY * cross.z();
                }
            } else if (count < navPath.poses.size()){
                count++;
            } else {
                driveX = 0;
                driveY = 0;
            }
            
            vel_msg.linear.x = driveX;
            vel_msg.linear.y = driveY;
            
            twistPublisher.publish(vel_msg);
            
            ros::spinOnce();
        
        }
    }
}


LocalPlanner::LocalPlanner() {
    
    // Setup subscriber to astar path
    pathSubscriber = nh.subscribe<nav_msgs::Path>(PATH_TOPIC, 1, &LocalPlanner::pathCallback, this);
    
    //Subscribe to lidar obstacle detection
    //lidarSubscriber = nh.subscribe<INSERT_MSG>(LIDAR_TOPIC, 1, &LocalPlanner::lidarCallback, this);
    
    //Setup twist publisher to cmdVelToJoints
    twistPublisher = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, 1, true);
    
    count = 0;
    
    received = FALSE;
}


void LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& pathIn){
    navPath.header = pathIn->header;
    navPath.poses = pathIn->poses;
    received = TRUE;
    count = 0;
}

void LocalPlanner::lidarCallback(){
    //TODO: Lidar handling
}
