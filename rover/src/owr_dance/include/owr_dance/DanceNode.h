/*
 * Date Started: 03/06/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: dance_node
 * ROS Package: owr_dance 
 * Purpose: Makes the rover dance 
 */

#ifndef DANCE_NODE_H
#define DANCE_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class DanceNode {
    public:
        DanceNode();
        void spin();
    private:
        ros::NodeHandle nh;
        ros::Publisher frontLeftSwervePub;
        ros::Publisher frontRightSwervePub;
        ros::Publisher armRotPub;
        

};

#endif