/*
 * Converts CMD_VEL vectors into arm joint positions
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017, 2018
 */


#include "armControl.hpp"
#include "cmdVelToArmJoints.hpp"
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

static ArmControl armControl;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_arm_joints");

    CmdVelToArmJoints cmdVelToArmJoints;
    cmdVelToArmJoints.run();
}

CmdVelToArmJoints::CmdVelToArmJoints() {
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    cmdArmJoySub = nh.subscribe<sensor_msgs::Joy>(
        TOPIC,
        1,
        &CmdVelToArmJoints::receiveArmVelMsg,
        this,
        transportHints);

    armUpper = nh.advertise<std_msgs::Float64>("/arm_top_controller/command", 1, true);
    armLower = nh.advertise<std_msgs::Float64>("/arm_bottom_controller/command", 1, true);
    armBaseRotatePub = nh.advertise<std_msgs::Float64>("/arm_base_rotate_controller/command", 1, true);
    // this topic needs to be checked
    clawGripPub = nh.advertise<std_msgs::Float64>("/claw_grip_controller/command", 1, true);
    // this topic needs to be checked
    clawTwistPub = nh.advertise<std_msgs::Float64>("/claw_rotate_controller/command", 1, true);

    armUpperActuator = 0;
    armLowerActuator = 0;
}

void CmdVelToArmJoints::run() {
    ros::Rate r(25);
    while (ros::ok()) {
        // Publish to all the publishers
        std_msgs::Float64 msg;
        msg.data = armUpperActuator;
        armUpper.publish(msg);
        r.sleep();
        msg.data = armLowerActuator;
        armLower.publish(msg);
        r.sleep();
        msg.data = armBaseRotate;
        armBaseRotatePub.publish(msg);
        r.sleep();
        msg.data = clawGrip;
        clawGripPub.publish(msg);
        r.sleep();
        msg.data = clawTwist;
        clawTwistPub.publish(msg);
        r.sleep();
        ros::spinOnce();
    }
}

void CmdVelToArmJoints::receiveArmVelMsg(const sensor_msgs::Joy::ConstPtr& joy) {
    // Convert the joystick message
    // into the messages for each of the arm publishers
    armJointVel armVels = armControl.convertJoystickMessageToJoints(joy);
    // Write the ouput to the private member variables of this class
    armUpperActuator = armVels.armUpperActuator;
    armLowerActuator = armVels.armLowerActuator;
    armBaseRotate = armVels.armBaseRotate;
    clawGrip = armVels.clawGrip;
    clawTwist = armVels.clawTwist;
    ROS_INFO("Joystick message left joystick y-axis = %f, right joystick y-axis = %f, dpad x-axis = %f, a button = %f, b button = %f, left shoulder = %f, right shoulder = %F, arm upper actuator = %f arm lower actuator = %f arm base rotate = %f claw grip = %f claw twist = %f",
        joy->axes[ARM_STICK_UPPER_UD], joy->axes[ARM_STICK_LOWER_UD],
        joy->axes[DPAD_LR],
        joy->buttons[A_BUTTON], joy->buttons[B_BUTTON],
        joy->buttons[LEFT_SHOULDER], joy->buttons[RIGHT_SHOULDER],
        armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist);
}
