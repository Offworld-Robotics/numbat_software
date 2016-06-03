/*
 * Date Started: 03/06/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: dance_node
 * ROS Package: owr_dance 
 * Purpose: Makes the rover dance 
 */

#include "owr_dance/DanceNode.h"

//TODO: make this a rosparam
#define BMP 120
#define RATE (1/(BMP/60.0))

int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_dance_node");
    
    DanceNode p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

DanceNode::DanceNode() {
    frontRightSwervePub = nh.advertise<std_msgs::Float64>("/front_right_wheel_axel_controller/command",1,false);
    frontLeftSwervePub  = nh.advertise<std_msgs::Float64>("/front_left_wheel_axel_controller/command",1,false); 
    armRotPub =           nh.advertise<std_msgs::Float64>("/arm_base_rotate_controller/command",1,false);  
    
    
}

void DanceNode::spin() {
    ros::Rate r(RATE);
    while(ros::ok()) {
        r.sleep();
        ros::spinOnce();
    }
}
