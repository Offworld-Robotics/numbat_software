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

#include "Cmd_Vel_Conversion.hpp"

/**
 * Rover's turn radius in meters.
 * TODO: test this
 */
static const double TURN_RADIUS = 1.0;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_cmd_vel_conversion");

    Cmd_Vel_Conversion conversion_node;
    conversion_node.run();
}

Cmd_Vel_Conversion::Cmd_Vel_Conversion() {
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(IN_TOPIC, 1, &Cmd_Vel_Conversion::receive_vel_msg, this, transportHints);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(OUT_TOPIC, 1, true);
}

void Cmd_Vel_Conversion::receive_vel_msg(const geometry_msgs::Twist::ConstPtr & vel_msg) {

    // if we're not turning, retransmit the message
    if(vel_msg->angular.z == 0.0L) {
        cmd_vel_pub.publish(vel_msg);
        return;
    }

    geometry_msgs::Twist out_msg;
    // I think that we want the "tangential velocity" for our y
    // Vt(y) = gama*radius
    out_msg.linear.y = vel_msg->angular.z * TURN_RADIUS;
    out_msg.linear.x = vel_msg->linear.x;
    cmd_vel_pub.publish(out_msg);

}

void Cmd_Vel_Conversion::run() {
    while(ros::ok()) {
        ros::spin();
    }
}
