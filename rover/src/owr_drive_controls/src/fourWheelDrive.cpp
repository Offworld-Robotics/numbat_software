/*
 * Orignal Author: Sajid Ibne Anower
 * Editors: Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Implementation of four wheel steering logic
 */


#include "fourWheelDrive.hpp"
#include <cmath>
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT


motorVels FourWheelDrive::steer(motorVels vels, double velMagnitude, double turnAngle) {
    motorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;
    output.frontLeftAng =  output.backLeftAng = turnAngle;
    /*
    if (turnAngle - M_PI < 2*M_PI) {
        output.frontRightAng = output.backLeftAng = turnAngle + M_PI;
    } else {
      	output.frontRightAng = output.backLeftAng = turnAngle - M_PI;
    } */
    return output;
}


motorVels FourWheelDrive::doVelTranslation(const geometry_msgs::Twist * velMsg) {
    motorVels output;
    double velMagnitude = getVelMagnitude(velMsg);
    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, velMsg->linear.x);
        int dir = getDir(velMsg->linear.x);
        // (turnAngle * dir) is the final normalised angle,
        // required to make the driving similar to that of a car
        double normalisedAngle = turnAngle * dir;
        output = steer(output, velMagnitude, normalisedAngle);
    } else {
        // y = 0
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
