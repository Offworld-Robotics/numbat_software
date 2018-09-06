/*
 * Date Started: 29/8/18
 * Original Author: Alan Nguyen
 * Editors: Edward Dai
 * ROS Node Name: crosbot_navigation
 * ROS Package: owr_positioning
 * Purpose: Crops subscribed images to a defined size and re-publishes them
 * This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017
 */

#ifndef CROSBOT_NAVIGATION_H
#define CROSBOT_NAVIGATION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#define TOPIC_MODE "/crosbot_navigation/mode"
#define TOPIC_GOAL "/crosbot_navigation/goal"

enum NavigationMode {
    PAUSE = 0,
    RESUME = 1,
};

class CrosbotNavigation {
    public:
        CrosbotNavigation();
        void run();
        void setMode(int32_t mode);
        void receiveModeMsg(const std_msgs::Int32::ConstPtr & mode_msg);
        void receiveGoalMsg(const geometry_msgs::Point::ConstPtr & goal_msg);

    private:
        ros::NodeHandle nh;
        ros::ServiceClient mode_client;
        ros::Subscriber mode_sub;
        ros::Subscriber goal_sub;
        
        std::string world_frame;
        int32_t mode;
        bool start;
};

#endif
