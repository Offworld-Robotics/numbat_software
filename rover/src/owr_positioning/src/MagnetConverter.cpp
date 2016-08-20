/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "MagnetConverter.h" 

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <list>
#include <cmath>
#include <stdio.h>

#define TOPIC "/owr/position"
//minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2 
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_magnet_node");
    
    MagnetConverter p(TOPIC);
    p.spin();
    ROS_INFO("finished!!!\n\n\n\n\n\n\n\n\n\n\n\n\n");
    return EXIT_SUCCESS;   
}

MagnetConverter::MagnetConverter(const std::string topic) {
    subscriber = node.subscribe("/mag", 5, &MagnetConverter::receiveMsg, this); // mangnet stuff
    publisher =  node.advertise<geometry_msgs::PoseStamped>("/owr/sensors/heading", 10);
}

geometry_msgs::Quaternion hamiltonProduct(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) {
    geometry_msgs::Quaternion ret;
    ret.w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
    ret.x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y);
    ret.y = (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x);
    ret.z = (q1.w * q2.z) + (q1.x * q2.y) + (q1.y * q2.x) - (q1.z * q2.w);
    return ret;
}

void MagnetConverter::receiveMsg(const boost::shared_ptr<geometry_msgs::Vector3 const> & msg) {
    //Normalized vector
    float norm = sqrt(pow(msg->x,2) + pow(msg->y,2) + pow(msg->z,2));
    geometry_msgs::Quaternion magQuart;
    magQuart.x = msg->x/norm;
    magQuart.y = -msg->y/norm;
    magQuart.z = msg->z/norm;
    magQuart.w = 0;
    //sensor_msgs::Imu imu;
    //geometry_msgs::Vector3 absDir = hamiltonProduct(magQuart,imu.orientation);
    //double heading = atan2(1,0) - atan2(absDir.y,absDir.x);
    double heading = atan2(magQuart.x,magQuart.y);

    ROS_INFO("recived");
    //We need orientation set
    //TODO: set valuews
    geometry_msgs::PoseStamped poseStamped;

    poseStamped.header.frame_id = "base_link";
    poseStamped.header.stamp = ros::Time(0);

    poseStamped.pose.position.x = 0;
    poseStamped.pose.position.y = 0;
    poseStamped.pose.position.z = 0;

    tf2::Quaternion q;
    q.setEuler(heading, 0, 0);

    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();

    publisher.publish(poseStamped);
}



//main loop
void MagnetConverter::spin() {
    while(ros::ok()) {
        ros::spinOnce();
        //ROS_INFO("spinning");
    }
}
