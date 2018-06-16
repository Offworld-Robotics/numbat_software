/*
 * Orignal Author: Sajid Ibne Anower
 * Editors: Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Implementation of four wheel steering logic
 */


#include "fourWheelSteer.hpp"
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT

fourWheelMotorVels fourWheelSteer(fourWheelMotorVels, double, double, int dir);
int getDir(double);

/**
 * sqrt(x^2 + y^2)
 */
double getHypotenuse(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

double getVelMagnitude(const geometry_msgs::Twist * velMsg) {
    double hypo = getHypotenuse(velMsg->linear.x, velMsg->linear.y);
    return fabs(hypo);
}

fourWheelMotorVels fourWheelSteer(fourWheelMotorVels vels,
                        double velMagnitude,
                        double turnAngle) {
    fourWheelMotorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;
    output.frontLeftAng =  output.backLeftAng = turnAngle;
    output.frontRightAng = output.backRightAng = -turnAngle;

    return output;
}

fourWheelMotorVels stop(fourWheelMotorVels vels) {
    fourWheelMotorVels output = vels;
    output.frontLeftMotorV = output.backLeftMotorV = 0;
    output.frontRightMotorV = output.backRightMotorV = 0;
    output.frontRightAng = output.frontLeftAng = 0;
    output.backRightAng = output.backLeftAng = 0;

    return output;
}

int getDir(double xVal) { return xVal >= 0 ? FRONT : BACK; }

fourWheelMotorVels doFourWheelTranslation(const geometry_msgs::Twist * velMsg) {
    fourWheelMotorVels output;
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
        output = fourWheelSteer(output, velMagnitude, normalisedAngle);
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
