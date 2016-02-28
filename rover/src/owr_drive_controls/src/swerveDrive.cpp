/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 19/02/2016
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 */

#include "swerveDrive.hpp"
#include <ros/ros.h>

#define ROVER_CENTRE_2_WHEEL_Y 0.40023
#define BACK_WHEEL_SPAN 0.80273
#define HALF_ROVER_WIDTH_X .27130
#define FRONT_W_2_BACK_W_X 0.54216

#define DEG90 M_PI_2


swerveMotorVels doVelTranslation ( const geometry_msgs::Twist * velMsg ) {
    swerveMotorVels output;
     //account for the special case where y=0
    if(velMsg->linear.y != 0) {
        const double turnAngle = atan2(velMsg->linear.y,velMsg->linear.x);
        const double rotationRadius = HALF_ROVER_WIDTH_X/sin(turnAngle);
        geometry_msgs::Vector3 rotationCentre;
        rotationCentre.x = -HALF_ROVER_WIDTH_X;
        rotationCentre.y = sqrt(pow(rotationRadius,2)-pow(HALF_ROVER_WIDTH_X,2));
        //magnitude velocity over rotation radius
        const double angularVelocity = (velMsg->linear.y / cos(turnAngle)) / rotationRadius;
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
        if(0 <= turnAngle && turnAngle <= M_PI) {
            output.frontLeftMotorV = closeFrontV;
            output.backLeftMotorV = closeBackV;
            output.frontRightMotorV = farFrontV;
            output.backRightMotorV = farBackV;
            output.frontLeftAng = closeFrontAng;
            output.frontRightAng = farFrontAng;
        } else {
            output.frontRightMotorV = closeFrontV;
            output.backRightMotorV = closeBackV;
            output.frontLeftMotorV = farFrontV;
            output.backLeftMotorV = farBackV;
            output.frontLeftAng = -farFrontAng;
            output.frontRightAng = -closeFrontAng;
        }
    } else {
        //y = 0
        output.frontLeftMotorV = output.backLeftMotorV = output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
    }
    return output;
}
