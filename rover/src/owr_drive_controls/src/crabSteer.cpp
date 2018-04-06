/*
 * Orignal Author: Sajid Ibne Anower
 * Editors:
 * Date Started: 3/04/2018
 * Purpose: Implementation of crab steering logic
 */


#include "crabSteer.hpp"
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define RIGHT 1
#define LEFT -RIGHT

crabMotorVels steerCrab(crabMotorVels,
                    double,
                    double,
                    double,
                    double,
                    double,
                    int dir);

int getDir(double);

/**
 * sqrt(x^2 + y^2)
 */
int getHypotenuse(int x, int y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}


int getVelMagnitude(const geometry_msgs::Twist * velMsg) {
    return fabs(getHypotenuse(velMsg->linear.x, velMsg->linear.y));
}

crabMotorVels steerCrab(crabMotorVels vels,
                     double closeFrontV,
                     double farFrontV,
                     double farBackV,
                     double closeBackV,
                     double turnAngle,
                     int dir) {

    crabMotorVels output = vels;
    output.frontLeftMotorV = closeFrontV * dir;
    output.backLeftMotorV = closeBackV * dir;
    output.frontRightMotorV = farFrontV * dir;
    output.backRightMotorV = farBackV * dir;
    output.frontLeftAng =  output.backLeftAng = (turnAngle * dir);
    output.frontRightAng = output.backRightAng = (turnAngle * dir);
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

int getDir(double turnAngle) {
    if(0 <= turnAngle && turnAngle <= M_PI) {
        return RIGHT;
    } else {
        return LEFT;
    }
}

crabMotorVels doCrabTranslation(const geometry_msgs::Twist * velMsg) {
    crabMotorVels output;

    // santity check, if the magnitude is close to zero stop
    int velMagnitude = getVelMagnitude(velMsg);
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    // account for the special case where y = 0
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, velMsg->linear.x);
        ROS_INFO("turnAngle %lf", turnAngle);

        // Setting all 
        double closeFrontV, farFrontV, farBackV, closeBackV;

        closeFrontV = farFrontV = velMagnitude;
        farBackV = closeBackV = velMagnitude;

        // if we are in reverse,
        // we just want to go round the same circle in the opposite direction
        if (velMsg->linear.x < 0) {
            // flip all the motorVs
            closeFrontV *=-1.0;
            farFrontV *=-1.0;
            farBackV *=-1.0;
            closeBackV *=-1.0;
        }

        // work out which side to favour
        int dir = getDir(turnAngle);
        output = steerCrab(output,
                        closeFrontV, farFrontV, farBackV, closeBackV,
                        turnAngle, dir);
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
