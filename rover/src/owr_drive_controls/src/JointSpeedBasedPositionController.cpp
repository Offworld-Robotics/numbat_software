/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointSpeedBasedPositionController.hpp"
#include <math.h>
#include <ros/ros.h>

#define VEL_INC 0.01
#define SMALL_INC 0.01

static inline caclShortestCircDelta(double a, double b) {
    double ab = (a - b);
    if (ab > M_PI && fabs(ab) < M_2_PI) {
        ab = (M_2_PI - ab);
    } else if (ab < -M_PI && fabs(ab) < M_2_PI) {
        ab = (M_2_PI + ab);
    }
    double ba = (b - a);
    if (ba > M_PI && fabs(ba) < M_2_PI) {
        ba = (M_2_PI - ba);
    } else if (ab < -M_PI && fabs(ab) < M_2_PI) {
        ba = (M_2_PI + ba);
    }
    double result = 0;
    if(fabs(ba) > fabs(ab)) {
        result  = ab;
    } else {
        result = ba;
    }
    
    if(result > M_2_PI) {
        ROS_ERROR("Invalid delta %f", result);
        return std::numeric_limits<double >::quiet_NaN();
    }
    return result;
}

JointSpeedBasedPositionController::JointSpeedBasedPositionController(double radius, double * gearRatio, int nGears,int minPWM, int maxPWM, int maxRPM) {
    wheelRadius = wheelRadiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    deltaPWM = maxPWMIn - minPWMIn;
    maxVelocity = (maxRPMIn * M_PI_2/SECONDS_IN_MINUTE);
    velocityRange = maxVelocity * 2;
    
    
    //inital values
    lastAimPosition = std::numeric_limits< double >::infinity();
    lastKnownPosition = std::numeric_limits< double >::infinity();
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
 * The co-ordinate system is such that the angal is 0 when the wheel is facing forward
 */
int JointSpeedBasedPositionController::posToPWM(double futurePos, double currentPos, double updateFrequency) {
    //check for invalid values
    if(fabs(futurePos) > M_PI_2) {
        ROS_ERROR("angle %f radians is not a valid position. Positions should be btween %f and %f",futurePos,M_PI_2,-M_PI_2);
        return -1;
    }
    
    double deltaT = 1/updateFrequency;
    //Step 1: Calculate the angular velocity currently
    double currentAngVel = calcShortestCircDelta(currentPos, lastKnownPosition)/deltaT;
    double targetAngularVel = currentAngVel;
    //Step 2: Calculate the direction to reach the desired position fastest (CW, CCW)
    double radialDistToTarget = futurePos - currentPos;
    
    //TODO: check limits and efficiencies here
    
    
    //Are we going in the right direction
    if(signbit(radialDistToTarget) == signbit(currentAngVel)) { //compares the sign of the two
        if(targetAngularVel > 0) {
            targetAngularVel = max(currentAngVel+VEL_INC,maxVelocity);
        } else if (targetAngularVel < 0) {
            targetAngularVel = min(currentAngVel-VEL_INC,-maxVelocity);
        }
    } else {
        //this will reverse the sign if it needs to be negative
        if(currentAngVel > 0) {
            targetAngularVel = maxVelocity;   
        } else {
            targetAngularVel = -maxVelocity;
        }
    }
    
    //Step 3: will we pass the point this turn
    double nextPosGuess = (targetAngularVel * updateFrequency) % M_2_PI;
    if(fabs(calcShortestCircDelta(nextPosGuess, currentPos)) >= fabs(calcShortestCircDelta(futurePos, currentPos))) {
        //decrease to a value that will get us just before it
        targetAngularVel = (((futurePos - SMALL_INC) - currentPos) / deltaT);
    }
    nextPosGuess = (targetAngularVel * updateFrequency);
    if(fabs(nextPosGuess) > M_PI_2 ) {
        
        //TODO: this is a danger
    }
    
    
    lastTargetVelocity = targetAngularVel;
    
    
    return 1000;
}
    
