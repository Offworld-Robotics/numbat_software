/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */

#include "JointVelocityController.hpp"
#include <math.h>
#include <limits>



//if this is not defined do it just based on pwm
#define DO_VEL_ADJUST


JointVelocityController::JointVelocityController(int minPWMIn, int maxPWMIn, int maxRPMIn, double wheelRadiusIn) {
    wheelRadius = wheelRadiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    deltaPWM = maxPWMIn - minPWMIn;
    velocityRange = (maxRPMIn * M_PI_2/SECONDS_IN_MINUTE) * 2;
    
    //inital values
    lastAngularVelocity = std::numeric_limits< double >::infinity();
}

/*
 * See notes at https://bluesat.atlassian.net/browse/OWRS-203 for maths explanation
 * Inputs:
 *        - targetVel  - the target point velocity of the wheel
 *        - currentVel - the current point velocity of the wheel 
 *        - currentAngle - the angle of the rover to the ground (incline)
 * 
 */
int JointVelocityController::velToPWM(double targetVel, double currentVel, double currentAngel) {
    int pwm;
    
    //w=v/r
    double targetAngularVelocity = targetVel/wheelRadius;
    double currentAngularVelocity = currentVel/wheelRadius;
    
    //ratio of velocity to pwm 
    //delta velocity / delta pwm
    double pwmVelRatio = (velocityRange) / (deltaPWM);
    pwm = (int)(targetAngularVelocity * pwmVelRatio) + minPWM + deltaPWM;
    
    #ifdef DO_VEL_ADJUST
        if(lastAngularVelocity !=  std::numeric_limits< double >::infinity() &&
            !FLOAT_EQL(currentAngularVelocity,lastAngularVelocity)) {
            int error = pwmVelRatio * (currentAngularVelocity - lastAngularVelocity);
            pwm += error;
        }
    #endif
    
    return pwm;
}


