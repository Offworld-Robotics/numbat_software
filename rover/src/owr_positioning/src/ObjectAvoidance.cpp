/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
#include "ObjectAvoidance.hpp"
#include <assert.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_avoidance");
    ObjectAvoidance ObjectAvoidance;
    ObjectAvoidance.run();
}

ObjectAvoidance::ObjectAvoidance() {

    //ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    sub = nh.subscribe<sensor_msgs::LaserScan>("joy",1, &ObjectAvoidance::scanCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/owr/drive/obj_avoid_twist", 10);
          
}

void ObjectAvoidance::run() {

    while(ros::ok()) {
        ros::spinOnce();
    }
}



void ObjectAvoidance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    

}


