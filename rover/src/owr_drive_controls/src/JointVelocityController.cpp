/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */

#include "JointVelocityController.hpp"

#define POWER_TORQUE_CONSTANT 9.549

JointVelocityController::JointVelocityController(double wheelRadiusIn, double maxPWMIn, double minPWMIn, double motorRPMIn, double motorBaseVoltageIn, double motorAmpsIn, double gearRatioIn) {
    wheelRadius = wheelRadiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    motorAmps = motorAmpsIn;
    motorRPM = motorAmpsIn;
    motorBaseVoltageIn = motorBaseVoltage;
    gearRatio = gearRatioIn;
}

/*
 * See notes at https://bluesat.atlassian.net/browse/OWRS-203 for maths explanation
 */
int JointVelocityController::velToPWM(double vel) {
    
    
    
}


