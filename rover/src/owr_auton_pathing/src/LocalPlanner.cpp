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
    ros::Rate loop_rate(10);
    return 0;
}


void LocalPlanner::run(){
    ros::Rate loop_rate(10);
    while(ros::ok()){
        
        double driveX = 0.0;
        double driveY = 0.0;
        
        // Message to be sent, defining the new direction of travel
        geometry_msgs::Twist vel_msg;
        
        //if no message has yet been received, wait, otherwise, use navPath to determine a twist
        if(!receivedPath){
            ros::spinOnce();
        } else if (count < navPath.poses.size()) {
            double roll, pitch, yaw;
            double driveX, driveY;
            double distance;
            
            double headingAngle, desiredAngle, resultantAngle;
            
            
            // Create a vector from currPosition to first <Pose> within navPath
            tf::Vector3 desiredVector( scaleMap(navPath.poses[count].pose.position.x) - scaleMap(currPosition.getOrigin().x()),
                                       scaleMap(navPath.poses[count].pose.position.y) - scaleMap(currPosition.getOrigin().y()), 
                                       0);
            
            
            //tf::Vector3 desiredVector( scaleMap(navPath.poses[count].pose.position.x) - scaleMap(2024.0),
            //                           scaleMap(navPath.poses[count].pose.position.y) - scaleMap(2024.0), 
            //                           0.0);
            
            //ROS_INFO("Inputs 2 %f, %d", navPath.poses[count].pose.position.x, count);
            //ROS_INFO("Desired Vector x: %f y: %f", desiredVector.getX(), desiredVector.getY());
            
            //TEST DATA**************************** TODO: remove
            //tf::Quaternion q( -0.002, -0.749, 0.250, 0.613);
            //currPosition.setRotation( q);
            
            // Find the orientation of the rover
            tf::Matrix3x3 m (currPosition.getRotation());
            
            m.getRPY( roll, pitch, yaw);
            
            //ROS_INFO("r: %f p: %f y: %f", 180.0*roll/M_PI, 180.0*pitch/M_PI, 180.0*yaw/M_PI);
            
            // Calculate a heading vector from the yaw (which is in radians)
            tf::Vector3 headingVector( cos(yaw), sin(yaw), 0.0);
            
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
                    driveY = 1.0;
                    //ROS_INFO("go right %f %f", driveX, driveY);
                } else {
                    driveX = MAX_SPEED;
                    driveY = -1.0;
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
        loop_rate.sleep();
    }
}


LocalPlanner::LocalPlanner() : nh(), mapSubscriber(nh, "map", 1), tfFilter(mapSubscriber, tfListener, "base_link", 1) {
    // Setup subscriber to astar path
    pathSubscriber = nh.subscribe<nav_msgs::Path>(PATH_TOPIC, 1, &LocalPlanner::pathCallback, this);
    
    //Subscribe to lidar obstacle detection
    lidarSubscriber = nh.subscribe<sensor_msgs::LaserScan>(LIDAR_TOPIC, 1, &LocalPlanner::scanCallback, this);
    
    
    testPublisher = nh.advertise<nav_msgs::Path>(TEST_PATH, 1, true);
    
    //Setup twist publisher to cmdVelToJoints
    twistPublisher = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, 1, true);
    
    // Does shit with map callback calls function maps
    tfFilter.registerCallback(boost::bind(&LocalPlanner::mapCallback, this, _1));
    
    count = 0;
    
    receivedPath = FALSE;
    receivedLaser = FALSE;
    //fLCollision = FALSE;
    //fRCollision = FALSE;
    //fCollision = FALSE;
    //bLCollision = FALSE;
    //bRCollision = FALSE;
}

// Receive a path from the astar
void LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& pathIn){
    navPath.header = pathIn->header;
    navPath.poses = pathIn->poses;
    
    //ROS_INFO("Inputs 1 %f", pathIn->poses[0].pose.position.x);
    receivedPath = TRUE;
    count = 0;
}

// Query the listener for a specific transform, that being:
// From: arg0 = "map" 
// To: arg1 = "base_link" 
// At the time: arg2 = girdData->header.stamp
// The object we put the resulting transform: arg3 = currPosition
void LocalPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData) {
    tfListener.lookupTransform("map","base_link",  ros::Time(), currPosition);
}

// Scale map from squares to metres
double LocalPlanner::scaleMap(double value){
    return value;
}

// For lidar handling, store the most recent scan
void LocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    laser.header = scan->header;
    laser.angle_min = scan->angle_min;
    laser.angle_max = scan->angle_max;
    laser.angle_increment = scan->angle_increment;
    laser.time_increment = scan->time_increment;
    laser.scan_time = scan->scan_time;
    laser.range_min = scan->range_min;
    laser.range_max = scan->range_max;
    
    //sensor_msgs::LaserScan objects;
    //laserFilter.update(*scan, objects);
    
    laser.ranges = scan->ranges;
    laser.intensities = scan->intensities;
    receivedLaser = TRUE;
    
    //LocalPlanner::lidarAvoidance();
}

/*

// Object avoidance function. Takes in the desired direction of movement,
// uses the laser Scan for object detection and adjusts the movement if a collision might occur
void LocalPlanner::lidarAvoidance( void){
    
    //fLFCollision = FALSE;
    fLHCollision = FALSE;
    fRHCollision = FALSE;
    //fRFCollision = FALSE;
    fCollision = FALSE;
    bRCollision = FALSE;
    bLCollision = FALSE;
    
    double angle = 0.0;
    
    int scanSize = laser.ranges.size();
    int indexCentre = - (laser.angle_min) / laser.angle_increment;
    int indexDiffR = indexCentre - ( DIFF_TURN_ANGLE / laser.angle_increment);
    int indexDiffL = indexCentre + ( DIFF_TURN_ANGLE / laser.angle_increment);
    int indexPIL = indexCentre + ( M_PI / laser.angle_increment);
    int indexPIR = indexCentre - ( M_PI / laser.angle_increment);
    int indexHalfPIL = indexCentre + ( M_PI / (2.0 * laser.angle_increment));
    int indexHalfPIR = indexCentre - ( M_PI / ( 2.0 * laser.angle_increment));
    
    
    for(int i = 0; i <= scanSize; i++){
        if( i <= indexPIR){
            angle = (indexPIR - i ) * laser.angle_increment;
            bRCollision = ( laser.range[i] <= 2.0 * MAX_TURN_RADIUS * cos(angle));
        } else if (i <= indexHalfPIR){
            angle = ( i - indexPIR) * laser.angle_increment;
            //fRFCollision = ( laser.range[i] <= 2.0 * MAX_TURN_RADIUS * cos(angle))
        } else if ( i < indexDiffR) {
            angle = ( i - indexPIR) * laser.angle_increment;
            fRHCollision = ( laser.range[i] <= 2.0 * MAX_TURN_RADIUS * cos(angle));
        } else if ( i < indexCentre){
            angle = ( i - indexPIR) * laser.angle_increment;
            fRHCollision = ( fRHCollision || ( laser.range[i] <= 2.0 * MAX_TURN_RADIUS * cos(angle)));
        } else if (i <= indexDiffL){
            
        } else if ( i < indexHalfPIL) {
            
        } else if ( i < indexDiffL){
            
        } else {
            
        }
    }
    
    
    if(valY >= 2 * MAX_TURN){
        //Detect if a left turn will result in a collision. This is done my claculating whether following
        //the current turn results in collision with an object.
        //TODO: add page on wiki explaining logic herein:
        //TODO: take into account laser.angle_min is negative
        //TODO: take into account destAngle beyond lidar range
        
        for(int i = (( laser.angle_min / laser.angle_increment) - ( DELTA_TURN_ANGLE / laser.angle_increment)); i < laser.ranges.size() && ((i * laser.angle_increment) <= (destAngle * M_PI) + (2 * DELTA_TURN_ANGLE); i++){
            if(i < (( laser.angle_min / laser.angle_increment)){
                collision = (laser.ranges[i] <= MIN_RANGE_OUTER_TURN);
            } else {
                collision = (laser.ranges[i] <= 2.0 * MAX_TURN * cos(M_PI/2.0 - (i * laser.angle_increment) - DELTA_TURN_ANGLE));
            }
        }
        
    } else if (valY < -2 * MAX_TURN){
        //Detect if a right turn will result in collision
        
        for(int i = (( laser.angle_min / laser.angle_increment)) + (destAngle * M_PI) - (2 * DELTA_TURN_ANGLE); i < (( laser.angle_min / laser.angle_increment) + ( DELTA_TURN_ANGLE / laser.angle_increment)); i++){
            if(i > (( laser.angle_min / laser.angle_increment)){
                collision = (laser.ranges[i] <= MIN_RANGE_OUTER_TURN);
            } else {
                collision = (laser.ranges[i] <= 2.0 * MAX_TURN * -cos(-M_PI/2.0 + (i * laser.angle_increment) - DELTA_TURN_ANGLE));
            }
        }
        
    } else {
        //Detect if straight forward will result in collision
        
        for (int i = (( laser.angle_min / laser.angle_increment)) - (M_PI / laser.angle_increment); i <= (( laser.angle_min / laser.angle_increment)) + (M_PI / laser.angle_increment) && i < laser.ranges.size(); i++){
            if(i < ( laser.angle_min / laser.angle_increment)){
                collision = (laser.ranges[i] < cos((( laser.angle_min / laser.angle_increment)) - (M_PI / laser.angle_increment) + i * laser.angle_increment)); //TODO: sign of cos result
            } else {
                collision = (laser.ranges[i] < cos((( laser.angle_min / laser.angle_increment)) - (M_PI / laser.angle_increment) + i * laser.angle_increment)); //TODO: continue if bothered
            }
        }
    }
} */
