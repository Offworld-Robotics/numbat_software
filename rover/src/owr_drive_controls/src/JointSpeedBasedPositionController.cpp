/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointSpeedBasedPositionController.hpp"

JointSpeedBasedPositionController:JointSpeedBasedPositionController(double wheelRadius, double maxPWM, double minPWM, double motorRPM, double motorBaseVoltage, double motorAmps, double gearRatioIn) {
    
}

/*
 * All positions are in radians
 * Parameters:
 *      futurePos: the absolute position in radians we want to reach
 *      currentPos: the position of the joint in radians at the current time
 *      lastPos: The postion of the joint one update frequency ago
 *      updateFrequency: The time gap between lastPos and currentPos in Hz
 * Returns:
 *      a pwm value to be passed to the motor
 */
int JointSpeedBasedPositionController::posToPWM(double futurePos, double currentPos, double lastPos, double updateFrequency ) {
    //Step 1: Calculate the angular velocity currently
    //Step 2: Calculate the direction to reach the desired position fastest (CW, CCW)
    //if we are going in the wrong direction change the direction
    //Step 3: Calculate if we will reach that position this cycle
    // If we are slow down so we meet it
    // Otherwise speed up and in the right direction
    return 1000;
}
    
