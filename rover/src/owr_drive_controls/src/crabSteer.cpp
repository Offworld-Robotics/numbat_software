/*
 * Orignal Author: Sajid Ibne Anower
 * Editors:
 * Date Started: 3/04/2018
 * Purpose: Implementation of crab steering logic
 */


#include "crabSteer.hpp"
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT

crabMotorVels steerCrab(crabMotorVels, double, double, int dir);
int getDir(double);

/**
 * sqrt(x^2 + y^2)
 */
double getHypotenuse(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

double getVelMagnitude(const geometry_msgs::Twist * velMsg) {
    double hypo = getHypotenuse(velMsg->linear.x, velMsg->linear.y);
    return fabs(hypo);
}

crabMotorVels steerCrab(crabMotorVels vels,
                        double velMagnitude,
                        double turnAngle) {
    crabMotorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;
    output.frontLeftAng =  output.backLeftAng = turnAngle;
    output.frontRightAng = output.backRightAng = turnAngle;
    return output;
}

crabMotorVels stop(crabMotorVels vels) {
    crabMotorVels output = vels;
    output.frontLeftMotorV = output.backLeftMotorV = 0;
    output.frontRightMotorV = output.backRightMotorV = 0;
    output.frontRightAng = output.frontLeftAng = 0;
    output.backRightAng = output.backLeftAng = 0;
    return output;
}

int getDir(double xVal) { return xVal >= 0 ? FRONT : BACK; }

crabMotorVels doCrabTranslation(const geometry_msgs::Twist * velMsg) {
    crabMotorVels output;

    double velMagnitude = getVelMagnitude(velMsg);
    int dir = getDir(velMsg->linear.x);
    const double turnAngle = atan2(velMsg->linear.y, velMsg->linear.x);
    double normalisedAngle = turnAngle * dir;

    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
        output.frontRightAng = output.frontLeftAng = normalisedAngle;
        output.backRightAng = output.backLeftAng = normalisedAngle;
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        // (turnAngle * dir) is the final normalised angle,
        // required to make the driving similar to that of a car
        double normalisedVel = dir * velMagnitude;
        output = steerCrab(output, normalisedVel, normalisedAngle);
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
