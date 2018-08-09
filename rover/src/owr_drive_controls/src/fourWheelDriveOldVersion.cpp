/*
 * Orignal Author: Harry J.E Day
 * Editors: Anita Smirnov
 * Date Started: 19/02/2016
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 * for a four wheel drive
 */

#include "fourWheelDrive.hpp"
#include <ros/ros.h>

#define ROVER_CENTRE_2_WHEEL_Y 0.40023
#define BACK_WHEEL_SPAN 0.80273
#define HALF_ROVER_WIDTH_X .27130
#define FRONT_W_2_BACK_W_X 0.54216
#define VEL_ERROR 0.01

#define DEG90 M_PI_2


fourWheelMotorVels doVelTranslationFour (const geometry_msgs::Twist * velMsg) {
    
    fourWheelMotorVels output;
    //santity check, if the magnitude is close to zero stop
    if(fabs(sqrt(pow(velMsg->linear.x, 2) + pow(velMsg->linear.y, 2))) < VEL_ERROR) {
        output.frontLeftMotorV = output.backLeftMotorV = output.frontRightMotorV = output.backRightMotorV = 0;
        output.frontRightAng = output.frontLeftAng = output.backRightAng = output.backLeftAng = 0;
     //account for the special case where y = 0
    } else if(fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y,velMsg->linear.x);
        const double rotationRadius = HALF_ROVER_WIDTH_X/sin(turnAngle);
        geometry_msgs::Vector3 rotationCentre;
        rotationCentre.x = -HALF_ROVER_WIDTH_X;
        rotationCentre.y = sqrt(pow(rotationRadius,2)-pow(HALF_ROVER_WIDTH_X,2));
        //magnitude velocity over rotation radius
        //const double angularVelocity = (velMsg->linear.y / cos(turnAngle)) / rotationRadius;
        const double angularVelocity = fabs(sqrt(pow(velMsg->linear.x, 2) + pow(velMsg->linear.y, 2))) / rotationRadius;
        ROS_INFO("turnAngle %lf, rotationRadius %lf, rotationCentre  %lf, %lf, %lf",turnAngle, rotationRadius, rotationCentre.x, rotationCentre.y, rotationCentre.z);
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
	double closeBackAng = DEG90-atan2(closeFrontR,FRONT_W_2_BACK_W_X);
	double farBackAng = DEG90-atan2(farFrontR,FRONT_W_2_BACK_W_X);
        
        //if we are in reverse, we just want to go round the same circle in the opposite direction (I think...)
        if(velMsg->linear.x < 0) {
            //flip all the motorVs
            closeFrontV *=-1.0;
            farFrontV *=-1.0;
            farBackV *=-1.0;
            closeBackV *=-1.0;
        }
        
        //work out which side to favour
        if (0 <= turnAngle && turnAngle <= M_PI) {
            output.frontLeftMotorV = closeFrontV;
            output.backLeftMotorV = closeBackV;
            output.frontRightMotorV = farFrontV;
            output.backRightMotorV = farBackV;
            output.frontLeftAng = closeFrontAng;
            output.frontRightAng = farFrontAng;
	    output.backLeftAng = closeBackAng;
	    output.backRightAng = farBackAng;
            ROS_INFO("right");
        } else {
            output.frontRightMotorV = -closeFrontV;
            output.backRightMotorV = -closeBackV;
            output.frontLeftMotorV = -farFrontV;
            output.backLeftMotorV = -farBackV;
            output.frontLeftAng = -farFrontAng;
            output.frontRightAng = -closeFrontAng;
	    output.backLeftAng = -closeBackAng;
	    output.backRightAng = -farBackAng;
            ROS_INFO("left");
        }
    } else {
        //y = 0
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = output.backLeftAng = output.backRightAng = 0;
    }
    return output;
}
