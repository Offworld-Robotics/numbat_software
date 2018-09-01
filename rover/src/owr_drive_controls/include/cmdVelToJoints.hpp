/*
 * Converts CMD_VEL vectors into joint positions
 * For now it is only used for the simulator, but could become main algorithm for steering bluetounge 2.0
 * Original Author: Harry J.E Day
 * Editors: Sajid Ibne Anower
 * Date Started: 8/02/2015
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 * This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2015
 */

#ifndef CMD_VEL_TO_JOINTS_H
#define CMD_VEL_TO_JOINTS_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "Drive.hpp"


#define TOPIC_VEL "/cmd_vel"
#define TOPIC_MODE "/cmd_mode"

enum driveMode {
    CRAB   = 0,
    FOUR   = 1,
    SWERVE = 2
};

class CmdVelToJoints {
    public:
        CmdVelToJoints();
        void run();
        driveMode getDriveMode();
        
    protected:
        void receiveDriveModeMsg(const std_msgs::Int16::ConstPtr& driveModeMsg);
        void receiveVelMsg(const geometry_msgs::Twist::ConstPtr& velMsg);

    private:
        ros::NodeHandle nh;
        ros::Subscriber cmdDriveModeSub;
        ros::Subscriber cmdVelSub;
        
        ros::Publisher frontLeftDrive;
        ros::Publisher frontRightDrive;
        ros::Publisher backLeftDrive;
        ros::Publisher backRightDrive;
        ros::Publisher frontLeftSwerve;
        ros::Publisher frontRightSwerve;
        ros::Publisher backLeftSwerve;
        ros::Publisher backRightSwerve;
        
        driveMode mode;
        double frontLeftMotorV, frontRightMotorV, backLeftMotorV, backRightMotorV;
        double frontLeftAng, frontRightAng, backLeftAng, backRightAng;
};

#endif // CMD_VEL_TO_JOINTS_H
