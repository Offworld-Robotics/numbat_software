/*
 * @author: (Orignal Author) Sajid Ibne Anower
 * @authors: ( Editors) Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Implementation of four wheel steering logic
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017
 */


#include "fourWheelDrive.hpp"
#include <cmath>
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT

#define FWDRIVE_LIMIT 0.45*M_PI/2

motorVels FourWheelDrive::steer(motorVels vels, double velMagnitude, double turnAngle) {
    motorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;

    // Capping the turnangle to 45% of M_PI/2
    if(turnAngle >= FWDRIVE_LIMIT) {
        turnAngle = FWDRIVE_LIMIT;
    } else if(turnAngle <= -FWDRIVE_LIMIT) {
        turnAngle = -FWDRIVE_LIMIT;
    }

    output.frontLeftAng = output.frontRightAng = turnAngle;
    output.backLeftAng = output.backRightAng = -turnAngle;
    return output;
}


motorVels FourWheelDrive::doVelTranslation(const geometry_msgs::Twist * velMsg) {
    motorVels output;
    double velMagnitude = getVelMagnitude(velMsg);
    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, fabs(velMsg->linear.x));
        int dir = getDir(velMsg->linear.x);
        // (turnAngle * dir) is the final normalised angle,
        // required to make the driving similar to that of a car
        double normalisedAngle = turnAngle * dir;
        output = steer(output, dir * velMagnitude, normalisedAngle);
    } else {
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
