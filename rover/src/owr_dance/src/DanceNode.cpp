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
#define BMP 120.0
#define RATE (BMP/60.0)

//45 degrees
#define ARM_MAX_ANGLE 0.785398
#define WHEEL_MAX_ANGLE (M_PI/2)

#define METER 4 // 4 

int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_dance_node");
    
    DanceNode p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

DanceNode::DanceNode() {
    frontRightSwervePub = nh.advertise<std_msgs::Float64>("/front_right_swerve_controller/command",1,false);
    frontLeftSwervePub  = nh.advertise<std_msgs::Float64>("/front_left_swerve_controller/command",1,false); 
    armRotPub =           nh.advertise<std_msgs::Float64>("/arm_base_rotate_controller/command",1,false);  
    
    
}

void DanceNode::spin() {
    ros::Rate r(RATE);
    int beat = 0;
    
    std_msgs::Float64 armPos;
    std_msgs::Float64 wheelPos;
    armPos.data = 0;
    wheelPos.data = 0;
    while(ros::ok()) {
        ROS_INFO("%d\n",beat+1);
        beat = (beat + 1)%METER;
        
        
        //on the first beat change the way we are moving the arm
        if(beat == 0) {
            if(armPos.data < 0) {
                armPos.data = ARM_MAX_ANGLE;
            } else {
                armPos.data = -ARM_MAX_ANGLE;
            }
            armRotPub.publish<std_msgs::Float64>(armPos);
        } 
        
        //On every odd beat move the wheels
        if(beat % 2) {
            if(wheelPos.data > 0) {
                wheelPos.data = -WHEEL_MAX_ANGLE;
            } else {
                wheelPos.data = WHEEL_MAX_ANGLE;
            }
            frontRightSwervePub.publish<std_msgs::Float64>(wheelPos);
            frontLeftSwervePub.publish<std_msgs::Float64>(wheelPos);
        }
        r.sleep();
        ros::spinOnce();
    }
}
