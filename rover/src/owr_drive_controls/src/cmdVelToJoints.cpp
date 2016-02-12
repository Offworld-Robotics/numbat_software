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
#define ROVER_CENTRE_2_WHEEL_Y 0.40023
#define BACK_WHEEL_SPAN 0.80273
#define HALF_ROVER_WIDTH_X .27130
#define FRONT_W_2_BACK_W_X 0.54216

#define DEG90 1.5708

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
    
    frontLeftMotorV =0;
    frontRightMotorV = 0;
    backLeftMotorV = 0;
    backRightMotorV = 0;
    frontLeftAng = 0;
    frontRightAng = 0;
}

void CmdVelToJoints::run() {
    while(ros::ok()) {
        ros::spinOnce();
        std_msgs::Float64 msg;
        //one side needs to be fliped so the joint velocity is relevant to the point velocity
        msg.data = frontLeftMotorV;
        frontLeftDrive.publish(msg);
        msg.data = -frontRightMotorV;
        frontRightDrive.publish(msg);
        msg.data = backLeftMotorV;
        backLeftDrive.publish(msg);
        msg.data = -backRightMotorV;
        backRightDrive.publish(msg);
        msg.data = frontLeftAng;
        frontLeftSwerve.publish(msg);
        msg.data = frontRightAng;
        frontRightSwerve.publish(msg);
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
    //account for the special case where y=0
    if(velMsg->linear.y != 0) {
        const double turnAngle = atan2(velMsg->linear.y,velMsg->linear.x);
        const double rotationRadius = HALF_ROVER_WIDTH_X/sin(turnAngle);
        geometry_msgs::Vector3 rotationCentre;
        rotationCentre.x = -HALF_ROVER_WIDTH_X;
        rotationCentre.y = sqrt(pow(rotationRadius,2)+pow(HALF_ROVER_WIDTH_X,2));
        const double angularVelocity = velMsg->linear.x / rotationRadius;
        ROS_INFO("turnAngle %lf, rotationRadius %lf, rotationCenter  %lf, %lf, %lf",turnAngle, rotationRadius, rotationCentre.x, rotationCentre.y, rotationCentre.z);
        //calculate the radiuses of each wheel about the rotation center
        //NOTE: if necisary this could be optimised
        double closeBackR = fabs(rotationCentre.y - ROVER_CENTRE_2_WHEEL_Y);
        double farBackR = fabs(rotationCentre.y + ROVER_CENTRE_2_WHEEL_Y);
        double closeFrontR = sqrt(pow(closeBackR,2) + pow(FRONT_W_2_BACK_W_X,2));
        double farFrontR = sqrt(pow(farBackR,2) + pow(FRONT_W_2_BACK_W_X,2));
        
        //V = wr
        double closeBackV = closeBackR * angularVelocity;
        double farBackV = farBackR * angularVelocity;
        double closeFrontV = closeFrontR * angularVelocity;
        double farFrontV = farFrontR * angularVelocity;
        
        //work out the front wheel angles
        double closeFrontAng = DEG90-atan2(closeBackR,FRONT_W_2_BACK_W_X);
        double farFrontAng = DEG90-atan2(farBackR,FRONT_W_2_BACK_W_X);
        
        //work out which side to favour
        if(0 <= turnAngle && turnAngle <= DEG90*2) {
            frontLeftMotorV = closeFrontV;
            backLeftMotorV = closeBackV;
            frontRightMotorV = farFrontV;
            backRightMotorV = farBackV;
            frontLeftAng = closeFrontAng;
            frontRightAng = farFrontAng;
        } else {
            frontRightMotorV = closeFrontV;
            backRightMotorV = closeBackV;
            frontLeftMotorV = farFrontV;
            backLeftMotorV = farBackV;
            frontLeftAng = farFrontAng;
            frontRightAng = closeFrontAng;
        }
    } else {
        //y = 0
        frontLeftMotorV = backLeftMotorV = frontRightMotorV = backRightMotorV = velMsg->linear.x;
        frontRightAng = frontLeftAng = 0;
    }
    
    
    
    
    
    
    
    
    
}

