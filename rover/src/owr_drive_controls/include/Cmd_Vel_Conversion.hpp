/*
 * @date: 27/08/18
 * @author: (original author) Harry J.E Day <harry@dayfamilyweb.com>
 * @authors: (editors) 
 * @details: Provides a conversion from a Twist message the uses linear.x and angular.z to one that uses linear.x and linear.y
 * @copydetails: This code is released under the MIT License
 * @copyright: Copyright BLUEsat UNSW 2018
 * ros_package: owr_drive_controls
 * ros_node: owr_cmd_vel_conversion
 */

#ifndef PROJECT_CMD_VEL_CONVERSION_HPP
#define PROJECT_CMD_VEL_CONVERSION_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define OUT_TOPIC "/cmd_vel"
#define IN_TOPIC "/cmd_vel/twist"

class Cmd_Vel_Conversion {

    public:
        Cmd_Vel_Conversion();
        /**
         * Starts the nodes spin loop
         */
        void run();

    protected:
        /**
         * Callback for the velocity message.
         *
         * Converts and republishes the message.
         *
         * @param vel_msg the message to convert.
         */
        void receive_vel_msg(const geometry_msgs::Twist::ConstPtr & vel_msg);

    private:
        ros::NodeHandle nh;
        ros::Subscriber cmd_vel_sub;
        ros::Publisher cmd_vel_pub;
};

#endif //PROJECT_CMD_VEL_CONVERSION_HPP
