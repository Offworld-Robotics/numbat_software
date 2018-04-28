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
double getHypotenuse(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

double getVelMagnitude(const geometry_msgs::Twist * velMsg) {
    double hypo = getHypotenuse(velMsg->linear.x, velMsg->linear.y);
    return fabs(hypo);
}

crabMotorVels steerCrab(crabMotorVels vels,
                        double velMagnitude,
                        double turnAngle) {

    double velMagnitudeVector = velMagnitude;
    crabMotorVels output = vels;
    output.frontLeftMotorV = velMagnitudeVector;
    output.backLeftMotorV = velMagnitudeVector;
    output.frontRightMotorV = velMagnitudeVector;
    output.backRightMotorV = velMagnitudeVector;
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

/**
 * @deprecated
 */
int getDir(double xVal) {
    return 1; // TODO: take this out
    ROS_INFO("x: %lf\n", xVal);
    /*if(0 <= turnAngle && turnAngle <= M_PI) {
        return FRONT;
    } else {
        return BACK;
    }*/
}

crabMotorVels doCrabTranslation(const geometry_msgs::Twist * velMsg) {
    crabMotorVels output;

    // If the magnitude is close to zero
    double velMagnitude = getVelMagnitude(velMsg);
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, velMsg->linear.x);
        ROS_INFO("turnAngle %lf", turnAngle);

        // Setting all equal to velMagnitude
        double closeFrontV, farFrontV, farBackV, closeBackV;

        closeFrontV = farFrontV = velMagnitude;
        farBackV = closeBackV = velMagnitude;

        // if we are in reverse,
        // we just want to go round the same circle in the opposite direction
        if (velMsg->linear.x < 0) {
            // flip all the motorVs
            closeFrontV *= -1.0;
            farFrontV *= -1.0;
            farBackV *= -1.0;
            closeBackV *= -1.0;
        }

        // work out which side to favour
        int dir = getDir(velMsg->linear.x);
	ROS_INFO("Direction: %d\n", dir);
        output = steerCrab(output, velMagnitude, turnAngle, dir);
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
