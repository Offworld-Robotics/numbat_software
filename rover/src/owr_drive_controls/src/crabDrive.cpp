/**
 * Converts CMD_VEL vectors into joint positions
 * @author: (Original Author) Sajid Ibne Anower
 * @authors: (Editors)
 * Date Started: 3/04/2018
 * Purpose: Implementation of crab steering logic
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017, 2018
 */


#include "crabDrive.hpp"
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT

motorVels CrabDrive::steer(motorVels vels, double velMagnitude, double turnAngle) {
    motorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;
    output.frontLeftAng =  output.backLeftAng = turnAngle;
    output.frontRightAng = output.backRightAng = turnAngle;
    return output;
}

motorVels CrabDrive::doVelTranslation(const geometry_msgs::Twist * velMsg) {
    motorVels output;
    double velMagnitude = getVelMagnitude(velMsg);
    const double turnAngle = atan2(velMsg->linear.y, fabs(velMsg->linear.x));
    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
        //output.frontRightAng = output.frontLeftAng = normalisedAngle;
        //output.backRightAng = output.backLeftAng = normalisedAngle;
        output.frontRightAng = output.frontLeftAng = turnAngle;
        output.backRightAng = output.backLeftAng = turnAngle;
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {        
        int dir = getDir(velMsg->linear.x);
        // (turnAngle * dir) is the final normalised angle,
        // required to make the driving similar to that of a car
        double normalisedVel = dir * velMagnitude;
        output = steer(output, normalisedVel, turnAngle);
    } else {
        // y = 0
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
