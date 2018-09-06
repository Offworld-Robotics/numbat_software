/*
 * Converts joystick vectors into arm joint positions
 * 
 * Original Author: Elliott Smiht
 * Editors:
 * Date Started: 2/6/18
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017, 2018
 */

#ifndef CMD_VEL_ARM_H
#define CMD_VEL_ARM_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "armControl.hpp"

#define TOPIC "/arm_joy" // need the joystick topic

class CmdVelToArmJoints {
    public:
        CmdVelToArmJoints();
        void run();
    protected:
        void receiveArmVelMsg(const sensor_msgs::Joy::ConstPtr& joy);

    private:
        ros::NodeHandle nh;
        ros::Subscriber cmdArmJoySub;

        ros::Publisher armUpper;
        ros::Publisher armLower;
        ros::Publisher armBaseRotatePub;
        ros::Publisher clawGripPub;
        ros::Publisher clawTwistPub;           
        double armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist;
};

#endif //CMD_VEL_ARM_H
