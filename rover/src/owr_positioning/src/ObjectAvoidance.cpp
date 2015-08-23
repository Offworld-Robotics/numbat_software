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
#include <tf/transform_listener.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_avoidance");
    ROS_INFO("object avoidance starting");
    ObjectAvoidance ObjectAvoidance;
    ROS_INFO("initialising...run");
    ObjectAvoidance.run();
}

ObjectAvoidance::ObjectAvoidance() : nh(), sub(nh, "/scan", 1),
    laserNotifierL(sub,listener, "left_front_wheel_hub", 1) {
    ROS_INFO("registering transform listner");
    laserNotifierL.registerCallback(
        boost::bind(&ObjectAvoidance::scanCallback, this, _1)
    );
    laserNotifierL.setTolerance(ros::Duration(0.01));
    //laserNotifierR.setTolerance(ros::Duration(0.01));
    //ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    //sub = nh.subscribe<sensor_msgs::LaserScan>("joy",1, &ObjectAvoidance::scanCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/owr/auton_twist", 10);
    
          
}

void ObjectAvoidance::run() {

    while(ros::ok()) {
        ros::spinOnce();
    }
}



void ObjectAvoidance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    #define DANGER_DIST 2.0
    #define ERROR_MARGIN 5
    ROS_INFO("received message");
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloudL;
    sensor_msgs::PointCloud cloudR;
    try{
        projector.transformLaserScanToPointCloud("left_front_wheel_hub",*scan, cloudL,listener);                        
        projector.transformLaserScanToPointCloud("right_front_wheel_hub",*scan, cloudL,listener); 
        int arrayLen = (scan->angle_max - scan->angle_min)/scan->angle_increment;
        int leftCount = 0;
        int rightCount = 0;
        for(int i =0; i<arrayLen; i++) {
            float distL = sqrt(pow(cloudL.points[i].x, 2) + pow(cloudL.points[i].y,2));
            float distR = sqrt(pow(cloudR.points[i].x, 2) + pow(cloudR.points[i].y,2));
            if(distL < DANGER_DIST) {
                distL++;
            }
            if(distR < DANGER_DIST) {
                distR ++;
            }
        }
        ROS_INFO("R %d L %d", rightCount, leftCount);
        float lf = 0.0;
        if (leftCount >= ERROR_MARGIN && leftCount > rightCount) {
            ROS_INFO("Turn right");
            lf = 1.0;
        } else if (rightCount >= ERROR_MARGIN) {
            ROS_INFO("Turn right");  
            lf = -1.0;
        }
        //Send twist message
        geometry_msgs::Twist vel;
        vel.linear.x = 1.0;
	    vel.linear.y = lf;
	    pub.publish(vel);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}


