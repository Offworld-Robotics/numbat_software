/*
 * Converts CMD_VEL vectors into arm joint positions
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 */




#include "armControl.hpp"
#include "cmdVelToArmJoints.hpp"
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_cmd_vel_2_arm_joints");
    CmdVelToArmJoints CmdVelToArmJoints;
    CmdVelToArmJoints.run();
}

CmdVelToArmJoints::CmdVelToArmJoints() {
     ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
     cmdArmVelSub = nh.subscribe<geometry_msgs::Twist>(TOPIC,1, &CmdVelToArmJoints::receiveArmVelMsg , this, transportHints);
     
    armUpper =  nh.advertise<std_msgs::Float64>("/arm_upper_actuator/command",1,true);
    armLower =  nh.advertise<std_msgs::Float64>("/arm_lower_actuator/command",1,true);
    
    
    armUpperActuator = 0;
    armLowerActuator = 0;
    
    
}

void CmdVelToArmJoints::run() {
    while(ros::ok()) {
        
        std_msgs::Float64 msg;
        msg.data = armUpperActuator;
        armUpper.publish(msg);
        msg.data = armLowerActuator;
	armLower.publish(msg);
	msg.data = armBaseRotate;
	armBaseRotatePub.publish(msg);
	msg.data = clawGrip;
	clawGripPub.publish(msg);
	msg.data = clawTwist;
	clawTwistPub.publish(msg);
        ros::spinOnce();
    }
}

void CmdVelToArmJoints::receiveArmVelMsg(const geometry_msgs::Twist::ConstPtr& stick) {
  armJointVel armVels = convertTwistMessagesToJoints(stick.get());
  armUpperActuator = armVels.armUpperActuator;
  armLowerActuator = armVels.armLowerActuator;
  armBaseRotate = armVels.armBaseRotate;
  clawGrip = armVels.clawGrip;
  clawTwist = armVels.clawTwist;
  ROS_INFO("Target arm position %f %f %f %f, arm upper actuator = %f arm lower actuator = %f arm base rotate = %f claw grip = %f claw twist = %f", stick->linear.x, stick->linear.y, stick->angular.x, stick->angular.y, armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist);
}

