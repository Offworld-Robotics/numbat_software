/*
 * Date Started: 13/06/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: orientation
 * ROS Package: owr_positioning
 * Purpose: Extracts the orientation of the rover from the odometry message
 */

#include "ExtractOrientation.h"

//TODO: make this a rosparam
#define BMP 120.0
#define RATE (BMP/60.0)

//45 degrees
#define ARM_MAX_ANGLE 0.785398
#define WHEEL_MAX_ANGLE ARM_MAX_ANGLE

#define METER 4 // 4 

int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_dance_node");
    
    ExtractOrientation p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

ExtractOrientation::ExtractOrientation() {
    
    quatMsgPub =  nh.advertise<geometry_msgs::Quaternion>("/orientation",1,false);
    odometrySub = nh.subscribe("/odometry/filtered",1,&ExtractOrientation::odometryCallback, this);
    
}


void ExtractOrientation::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::Quaternion q;
    q = msg->pose.pose.orientation;
    quatMsgPub.publish<geometry_msgs::Quaternion>(q);
}

void ExtractOrientation::spin() {
    ros::spin();
}
