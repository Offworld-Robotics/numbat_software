/*
 * Converts CMD_VEL vectors into joint positions
 * For now it is only used for the simulator, but could become main algorithm for steering bluetounge 2.0
 * Original Author: Harry J.E Day
 * Editors:
 * Date Started: 8/02/2015
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 */

/*Constants that are needed for this (eventually we should probably calculate these from transforms):

    ROVER_CENTER_2_WHEEL_Y := 400.23mm == 0.40023 (distance from base_link_origin to back_right_wheel shaft)
    BACK_WHEEL_SPAN :=802.73mm == 0.80273
    HALF_ROVER_WIDTH_X := 271.30mm == .27130 (distance from base_link_origin to back_right_wheel shaft)
    FRONT_W_2_BACK_W_X := 542.16mm == 0.54216
*/

#include "cmdVelToJoints.hpp"
#include "crabDrive.hpp"
#include "fourWheelDrive.hpp"
#include "swerveDrive.hpp"
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

static CrabDrive crabDrive;
static FourWheelDrive fourWheelDrive;
static SwerveDrive swerveDrive;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_cmd_vel_2_joints");
    CmdVelToJoints cmdVelToJoints;
    cmdVelToJoints.run();
}

CmdVelToJoints::CmdVelToJoints() {
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    cmdDriveModeSub = nh.subscribe<std_msgs::Int16>(TOPIC_MODE, 1, &CmdVelToJoints::receiveDriveModeMsg, this, transportHints);
    cmdVelSub = nh.subscribe<geometry_msgs::Twist>(TOPIC_VEL, 1, &CmdVelToJoints::receiveVelMsg, this, transportHints);
     
    frontLeftDrive = nh.advertise<std_msgs::Float64>("/front_left_wheel_axel_controller/command",1,true);
    frontRightDrive = nh.advertise<std_msgs::Float64>("/front_right_wheel_axel_controller/command",1,true);
    backLeftDrive = nh.advertise<std_msgs::Float64>("/back_left_wheel_axel_controller/command",1,true);
    backRightDrive = nh.advertise<std_msgs::Float64>("/back_right_wheel_axel_controller/command",1,true);
    frontLeftSwerve = nh.advertise<std_msgs::Float64>("/front_left_swerve_controller/command",1,true);
    frontRightSwerve = nh.advertise<std_msgs::Float64>("/front_right_swerve_controller/command",1,true);
    backLeftSwerve = nh.advertise<std_msgs::Float64>("/back_left_swerve_controller/command",1,true);
    backRightSwerve = nh.advertise<std_msgs::Float64>("/back_right_swerve_controller/command",1,true);
    
    mode = CRAB; // default to swerve mode
    frontLeftMotorV = 0;
    frontRightMotorV = 0;
    backLeftMotorV = 0;
    backRightMotorV = 0;
    frontLeftAng = 0;
    frontRightAng = 0;
    backLeftAng = 0;
    backRightAng = 0;
}

void CmdVelToJoints::run() {
    ros::Rate rate(60); // placeholder value
    while(ros::ok()) {
        std_msgs::Float64 msg;
        msg.data = frontRightMotorV;
        frontRightDrive.publish(msg);
        rate.sleep();
	msg.data = backRightMotorV;
        backRightDrive.publish(msg);
        rate.sleep();
        msg.data = frontLeftAng;
        frontRightSwerve.publish(msg);
        rate.sleep();
        
	    // one side needs to be fliped so the joint velocity is relevant to the point velocity
        if (mode == SWERVE) { // need to flip for swerve mode?
            // don't publish back wheel angles for swerve mode
            msg.data = -1.0 * frontLeftMotorV;
            frontLeftDrive.publish(msg);
            rate.sleep();

	    msg.data = -1.0 * backLeftMotorV;
            backLeftDrive.publish(msg);
            rate.sleep();
	    msg.data = -1.0 * frontRightAng;
            frontLeftSwerve.publish(msg);
        } else { // otherwise publish as normal
            msg.data = frontLeftMotorV;
            frontLeftDrive.publish(msg);
            rate.sleep();

	    msg.data = backLeftMotorV;
            backLeftDrive.publish(msg);
            rate.sleep();

            msg.data = frontRightAng;
            frontLeftSwerve.publish(msg);
            rate.sleep();

            msg.data = backLeftAng;
            backLeftSwerve.publish(msg);
            rate.sleep();

            msg.data = backRightAng;
            backRightSwerve.publish(msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

driveMode CmdVelToJoints::getDriveMode() {
    return mode;
}

void CmdVelToJoints::receiveDriveModeMsg(const std_msgs::Int16::ConstPtr& driveModeMsg) {
    mode = (driveMode) driveModeMsg->data;
    ROS_INFO("Drive mode = %d", (int) mode);
}

/*
 * Expectations (none automated tests)
 * Casse when linear.y is 0
 *      velocity of all motors = velMsg.x
 *      motor angles all = 0
 * 
 */

//for a full explanation of this logic please see the scaned notes at
//https://bluesat.atlassian.net/browse/OWRS-203
void CmdVelToJoints::receiveVelMsg(const geometry_msgs::Twist::ConstPtr& velMsg) {
    motorVels vels;  
    switch(mode) {
        case CRAB:
            vels = crabDrive.doVelTranslation(velMsg.get());
            ROS_INFO("crab drive");
            break;
        case FOUR:
            vels = fourWheelDrive.doVelTranslation(velMsg.get());
            ROS_INFO("four wheel drive");
            break;
        case SWERVE:
            vels = swerveDrive.doVelTranslation(velMsg.get());
            ROS_INFO("swerve drive");
            break;
    }
    frontLeftMotorV = vels.frontLeftMotorV;
    frontRightMotorV = vels.frontRightMotorV;
    backLeftMotorV = vels.backLeftMotorV;
    backRightMotorV = vels.backRightMotorV;
    frontLeftAng = vels.frontLeftAng;
    frontRightAng = vels.frontRightAng;
    backLeftAng = vels.backLeftAng;
    backRightAng = vels.backRightAng;
    ROS_INFO(
        "target %f,%f fl %f, fr %f, bl %f, br %f, fls %f, frs %f bls %f brs %f",
        velMsg->linear.x, velMsg->linear.y, frontLeftMotorV, frontRightMotorV, backLeftMotorV, backRightMotorV, frontLeftAng, frontRightAng, backLeftAng, backRightAng);
}
