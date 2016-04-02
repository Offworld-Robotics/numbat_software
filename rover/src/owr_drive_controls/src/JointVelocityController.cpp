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
// #define DO_VEL_ADJUST


JointVelocityController::JointVelocityController(int minPWMIn, int maxPWMIn, int maxRPMIn, double wheelRadiusIn,const double * gearsIn,const int nGearsIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
    wheelRadius = wheelRadiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    deltaPWM = maxPWMIn - minPWMIn;
    maxVelocity = (maxRPMIn * M_2_PI/SECONDS_IN_MINUTE);
    velocityRange = maxVelocity * 2;
    
    //inital values
    lastAngularVelocity = std::numeric_limits< double >::infinity();
    
    gears = gearsIn;
    nGears = nGearsIn;
    lastPWM = minPWMIn + deltaPWM;
}

/*
 * See notes at https://bluesat.atlassian.net/browse/OWRS-203 for maths explanation
 * Inputs:
 *        - targetVel  - the target point velocity of the wheel
 *        - currentVel - the current point velocity of the wheel 
 *        - currentAngle - the angle of the rover to the ground (incline)
 * 
 */
int JointVelocityController::velToPWM(double targetVel, double currentVel) {
    int pwm;
    printf("Vel target vel  %f, current vel %f\n", targetVel, currentVel);
//     printf("nGears %d", nGears);
    
    
    //w=v/r
    double targetAngularVelocity = targetVel/wheelRadius;
    double currentAngularVelocity = currentVel/wheelRadius;
    
    for(int i = 0; i < nGears; i++) {
        //TODO: optimise this
//         printf("i = %d", i);
        targetAngularVelocity /= gears[i];
        currentAngularVelocity /= gears[i];
    }
    //ratio of velocity to pwm 
    //delta velocity / delta pwm
    double pwmVelRatio =  (deltaPWM) / (velocityRange);
    printf("pwmVelRatio %f, velocityRange %f\n", pwmVelRatio, velocityRange);
//     printf("minPWM %d, deltaPWM %d\n", minPWM, deltaPWM);
    //correct for maximums
    targetAngularVelocity =  std::min(targetAngularVelocity, maxVelocity);
    targetAngularVelocity =  std::max(targetAngularVelocity, -maxVelocity);
    pwm = (int)(targetAngularVelocity * pwmVelRatio) + minPWM + (deltaPWM/2);
    
    #ifdef DO_VEL_ADJUST
        if(lastAngularVelocity !=  std::numeric_limits< double >::infinity() &&
            !FLOAT_EQL(currentAngularVelocity,lastAngularVelocity)) {
            int error = pwmVelRatio * (currentAngularVelocity - lastAngularVelocity);
            pwm += error;
        }
    #endif
    lastAngularVelocity = targetAngularVelocity;
    lastPWM = lastPWM;
//     printf("pwm %d\n", pwm);
    return pwm;
}


int JointVelocityController::velToPWM ( double currentVel ) {
    //safe value
    if (isnan(requestedValue)) {
            lastPWM =  minPWM + deltaPWM/2;
            return lastPWM;
    }
    return velToPWM(requestedValue, currentVel);
}


jointInfo JointVelocityController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime) {
    jointInfo info;
    info.position = 0.0; //TODO: estimate this
    info.velocity = lastAngularVelocity; //TODO: extrapolate based on delta
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}


