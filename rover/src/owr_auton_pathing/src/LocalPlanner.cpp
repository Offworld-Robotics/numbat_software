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
        
        //nav_msgs::Path testPath;
        
        //testPath.poses.resize(1);
        
        //geometry_msgs::PoseStamped thisPose;
        
        //thisPose.pose.position.x = 2024;
        //thisPose.pose.position.y = 2024;
        
        //testPath.poses[0] = thisPose;
        
        //testPublisher.publish(testPath);
        
        double driveX = 0.0;
        double driveY = 0.0;
        
        // Message to be sent, defining the new direction of travel
        geometry_msgs::Twist vel_msg;
        
        //if no message has yet been received, wait, otherwise, use navPath to determine a twist
        if(!received){
            ros::spinOnce();
        } else if (count < navPath.poses.size()) {
            double roll, pitch, yaw;
            double driveX, driveY;
            double distance;
            
            double headingAngle, desiredAngle, resultantAngle;
            
            //TODO: Insert local Lidar obstacle handling
            
            
            // Create a vector from currPosition to first <Pose> within navPath
            //tf::Vector3 desiredVector( scaleMap(navPath.poses[count].pose.position.x) - scaleMap(currPosition.getOrigin().x()),
            //                           scaleMap(navPath.poses[count].pose.position.y) - scaleMap(currPosition.getOrigin().y()), 
            //                           0);
            
            
            tf::Vector3 desiredVector( scaleMap(navPath.poses[count].pose.position.x) - scaleMap(2024.0),
                                       scaleMap(navPath.poses[count].pose.position.y) - scaleMap(2024.0), 
                                       0.0);
            
            //ROS_INFO("Inputs 2 %f, %d", navPath.poses[count].pose.position.x, count);
            //ROS_INFO("Desired Vector x: %f y: %f", desiredVector.getX(), desiredVector.getY());
            
            // Find the orientation of the rover
            tf::Matrix3x3 m(currPosition.getRotation());
            
            m.getRPY( roll, pitch, yaw);
            
            // Calculate a heading vector from the yaw (which is in radians)
            tf::Vector3 headingVector( cos(yaw), sin(yaw), 0);
            
            //ROS_INFO("Heading Vector x: %f y: %f", headingVector.getX(), headingVector.getY());
            
            //Calculate the angle between the two vectors (-pi,pi]:
            desiredAngle = std::atan2(desiredVector.getY(),desiredVector.getX());
            headingAngle = std::atan2(headingVector.getY(),headingVector.getX());
            
            //ROS_INFO("DANGLE %f", 180.0*desiredAngle/M_PI);
            //ROS_INFO("HANGLE %f", 180.0*headingAngle/M_PI);
            
            // (-2pi, 2pi], then convert to degrees (-180,180]
            resultantAngle = 180.0 * (desiredAngle - headingAngle)/M_PI;
            
            // Ensure the resultantAngle lies in (-180, 180]. Equivalent of using mod
            // function but avoids possible issue that mod function will influence sign of result
            resultantAngle += (resultantAngle>180.0) ? -360.0 : (resultantAngle<-180) ? 360.0 : 0.0;
            
            // Take as fraction, (-1,1]
            resultantAngle = resultantAngle/180.0;
            
            
            // Take the cross-product of the desired and heading vectors to determine turning direction
            tf::Vector3 cross = headingVector.cross(desiredVector);
            
            
            distance = desiredVector.length();
            
            //ROS_INFO("distance %f", distance);
            //ROS_INFO("angle %f", resultantAngle);
            
            //Normalise vector
            //TODO: remove if unneeded
            cross.normalize();
            
            if(distance > MIN_DISTANCE_TO_GOAL){
                
                // The result of the cross product will be a vector normal to the 2 vectors
                if( resultantAngle == 0.0) {
                    driveX = MAX_SPEED;
                    driveY = 0.0;
                    //ROS_INFO("go forwards %f %f", driveX, driveY);
                } else if (resultantAngle < 0.0) {
                    driveX = MAX_SPEED;
                    driveY = -1.0;
                    //ROS_INFO("go right %f %f", driveX, driveY);
                } else {
                    driveX = MAX_SPEED;
                    driveY = 1.0;
                    //ROS_INFO("go left %f %f", driveX, driveY);
                }
                
                //ROS_INFO("preadjust %f %f, %f, %f, %f", driveX, driveY, resultantAngle, fabs(resultantAngle), MAX_TURN);
                
                // Adjust turn to be relative to desired vector
                if( fabs( resultantAngle) <= MAX_TURN){
                    driveY = 2.0 * driveY * fabs(resultantAngle);
                    //ROS_INFO("adjust %f %f", driveX, driveY);
                }
                vel_msg.linear.x = driveX;
                vel_msg.linear.y = driveY;
            } else {
                // We have got close enough to current pose, begin navigation to next pose
                count++;
                driveX = 0;
                driveY = 0;
                //ROS_INFO("next pose %f %f", driveX, driveY);
                vel_msg.linear.x = driveX;
                vel_msg.linear.y = driveY;
            } 
        } else {
            // We have reached goal, DONT MOVE!!! 
            //ROS_INFO(" WE THERE");
            driveX = 0;
            driveY = 0;
            vel_msg.linear.x = driveX;
            vel_msg.linear.y = driveY;
        }
        //ROS_INFO("output %f %f", driveX, driveY);
        
        twistPublisher.publish(vel_msg);
        
        ros::spinOnce();
    }
}


LocalPlanner::LocalPlanner() : nh(), mapSubscriber(nh, "map", 1), tfFilter(mapSubscriber, tfListener, "base_link", 1) {
    // Setup subscriber to astar path
    pathSubscriber = nh.subscribe<nav_msgs::Path>(PATH_TOPIC, 1, &LocalPlanner::pathCallback, this);
    
    //Subscribe to lidar obstacle detection
    //lidarSubscriber = nh.subscribe<INSERT_MSG>(LIDAR_TOPIC, 1, &LocalPlanner::lidarCallback, this);
    
    
    testPublisher = nh.advertise<nav_msgs::Path>(TEST_PATH, 1, true);
    
    //Setup twist publisher to cmdVelToJoints
    twistPublisher = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, 1, true);
    
    // Does shit with map callback calls function maps
    tfFilter.registerCallback(boost::bind(&LocalPlanner::mapCallback, this, _1));
    
    count = 0;
    
    received = FALSE;
}

// Receive a path from the astar
void LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& pathIn){
    navPath.header = pathIn->header;
    navPath.poses = pathIn->poses;
    
    ROS_INFO("Inputs 1 %f", pathIn->poses[0].pose.position.x);
    received = TRUE;
    count = 0;
}

// Query the listener for a specific transform, that being:
// From: arg0 = "map" 
// To: arg1 = "base_link" 
// At the time: arg2 = girdData->header.stamp
// The object we put the resulting transform: arg3 = currPosition
void LocalPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData) {
    tfListener.lookupTransform("map","base_link", gridData->header.stamp, currPosition);
}

// Scale map from squares to metres
double LocalPlanner::scaleMap(double value){
    return 20 * (value + 101.25);
}

// For lidar handling
void LocalPlanner::lidarCallback(){
    //TODO: Lidar handling
}
