/*
 * Date Started: 13/06/16
 * Updated: 2019-09-08
 * Original Author: Harry J.E Day
 * Editors: Michael Lloyd
 * ROS Node Name: orientation
 * ROS Package: owr_positioning
 * Purpose: Extracts the orientation of the rover from the odometry message
 */

#include "ExtractOrientation.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "extract_orientation");
    ExtractOrientation p;
    p.spin();
    return EXIT_SUCCESS;   
}

ExtractOrientation::ExtractOrientation() {
    quatMsgPub =  nh.advertise<geometry_msgs::Quaternion>("/orientation", 1, false);
    odometrySub = nh.subscribe("/odometry/filtered", 1, &ExtractOrientation::odometryCallback, this);
    
}


void ExtractOrientation::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::Quaternion q;
    q = msg->pose.pose.orientation;
    quatMsgPub.publish<geometry_msgs::Quaternion>(q);
}

void ExtractOrientation::spin() {
    ros::spin();
}
