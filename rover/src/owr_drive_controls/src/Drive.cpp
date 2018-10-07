/**
 * Date Started: 9/8/18
 * @author: Alan Nguyen
 * @authors: (Editors) [Editor 1], [Editor 2]
 * @description: Base class for common drive system code
 * @copyright: This code is released under the MIT License. Copyright BLUEsat UNSW, 2018
 */
#include "Drive.hpp"

#define FRONT 1
#define BACK -FRONT

motorVels Drive::stop(motorVels vels) {
    motorVels output = vels;
    output.frontLeftMotorV = output.backLeftMotorV = 0;
    output.frontRightMotorV = output.backRightMotorV = 0;
    output.frontRightAng = output.frontLeftAng = 0;
    output.backRightAng = output.backLeftAng = 0;
    return output;
}

double Drive::getVelMagnitude(const geometry_msgs::Twist * velMsg) {
    double hypo = getHypotenuse(velMsg->linear.x, velMsg->linear.y);
    return fabs(hypo);
}

/**
 * sqrt(x^2 + y^2)
 */
double Drive::getHypotenuse(double x, double y) { 
    return sqrt(pow(x, 2) + pow(y, 2)); 
}

int Drive::getDir(double xVal) { 
    return xVal >= 0 ? FRONT : BACK;
}


