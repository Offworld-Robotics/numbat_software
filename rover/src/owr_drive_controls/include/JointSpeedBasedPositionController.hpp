/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointVelocityController.hpp"

class JointSpeedBasedPositionController : public JointVelocityController {
    JointSpeedBasedPositionController(double wheelRadius, double maxPWM, double minPWM, double motorRPM, double motorBaseVoltage, double motorAmps, double gearRatioIn);
    int posToPWM(double futurePos, double currentPos, double lastPos, double updateFrequency );
    
}