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


#include "swerveDrive.hpp"
#include "cmdVelToJoints.hpp"
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_cmd_vel_2_joints");
    CmdVelToJoints CmdVelToJoints;
    CmdVelToJoints.run();
}

CmdVelToJoints::CmdVelToJoints() {
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    cmdVelSub = nh.subscribe<geometry_msgs::Twist>(TOPIC,1, &CmdVelToJoints::reciveVelMsg , this, transportHints);
     
    frontLeftDrive = nh.advertise<std_msgs::Float64>("/front_left_wheel_axel_controller/command",1,true);
    frontRightDrive = nh.advertise<std_msgs::Float64>("/front_right_wheel_axel_controller/command",1,true);
    backLeftDrive = nh.advertise<std_msgs::Float64>("/back_left_wheel_axel_controller/command",1,true);
    backRightDrive = nh.advertise<std_msgs::Float64>("/back_right_wheel_axel_controller/command",1,true);
    frontLeftSwerve = nh.advertise<std_msgs::Float64>("/front_left_swerve_controller/command",1,true);
    frontRightSwerve = nh.advertise<std_msgs::Float64>("/front_right_swerve_controller/command",1,true);
    backLeftSwerve = nh.advertise<std_msgs::Float64>("/back_left_swerve_controller/command",1,true);
    backRightSwerve = nh.advertise<std_msgs::Float64>("/back_right_swerve_controller/command",1,true);
    
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
    while(ros::ok()) {
        
        std_msgs::Float64 msg;
        //one side needs to be fliped so the joint velocity is relevant to the point velocity
        msg.data = -1.0* frontLeftMotorV;
        frontLeftDrive.publish(msg);
        msg.data =  frontRightMotorV;
        frontRightDrive.publish(msg);
        msg.data = -1.0 * backLeftMotorV;
        backLeftDrive.publish(msg);
        msg.data =  backRightMotorV;
        backRightDrive.publish(msg);
        msg.data = frontRightAng*-1.0;
        frontLeftSwerve.publish(msg);
        msg.data = frontLeftAng;
        frontRightSwerve.publish(msg);
        msg.data = backLeftAng;
        backLeftSwerve.publish(msg);
        msg.data = backRightAng;
        backRightSwerve.publish(msg);
        ros::spinOnce();
    }
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
void CmdVelToJoints::reciveVelMsg ( const geometry_msgs::Twist::ConstPtr& velMsg ) {
    swerveMotorVels vels = doVelTranslation(velMsg.get());
    frontLeftMotorV = vels.frontLeftMotorV;
    frontRightMotorV = vels.frontRightMotorV;
    backLeftMotorV = vels.backLeftMotorV;
    backRightMotorV = vels.backRightMotorV;
    frontLeftAng = vels.frontLeftAng;
    frontRightAng = vels.frontRightAng;

    ROS_INFO("target %f,%f,%f. fl %f, fr %f, bl %f, br %f, fls %f, frs %f", velMsg->linear.x, velMsg->linear.y, velMsg->linear.z, frontLeftMotorV, frontRightMotorV, backLeftMotorV, backRightMotorV, frontLeftAng, frontRightAng);
}

