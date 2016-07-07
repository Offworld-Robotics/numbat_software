/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointSpeedBasedPositionController.hpp"
#include <math.h>
#include <ros/ros.h>

#define VEL_INC 0.025 
#define SMALL_INC 0.001
#define VEL_ERROR SMALL_INC

//if this is not defined do it just based on pwm
// #define DO_VEL_ADJUST

//coverts a radian postion so that -PI <= pos <= PI
static inline double posRangeConvert(double pos) {
//     printf("start pos %f\n", pos);
    while (pos > M_PI) {
        pos = -(2*M_PI - pos);
//         printf("pos %f\n",pos);
    } 
    while (pos < -M_PI) {
       pos = 2*M_PI + pos;
    }
    return pos;
}

static inline double signFMod(double v, double mod) {
    return v - floor(v/mod) * mod;
}


//Condition: -PI/2 <= a <= PI/2, -PI/2 <= b <= PI/2
static inline double calcShortestCircDelta(double a, double b) {
    
//     double ab = (a - b);
//     
//     //we want numbers to be in the range 
// //     if(
// //     if ( (ab + FLOATING_PT_ERROR) >= M_PI && fabs(ab) < (M_PI * 2)) {
// //         ab = ((M_PI * 2) - ab);
// //     } else if (ab < -M_PI && fabs(ab) < (M_PI * 2)) {
// //         ab = ((M_PI * 2) + ab);
// //     } else {
// //         printf("error\n");
// //     }
//     double ba = (b - a);
// //     if (ba > M_PI && fabs(ba) < (M_PI * 2)) {
// //         ba = ((M_PI * 2) - ba);
// //     } else if (ab < -M_PI && fabs(ab) < (M_PI * 2)) {
// //         ba = ((M_PI * 2) + ba);
// //     } else {
// //         printf("error\n");
// //     }
    
//     double ab = (a - b);
//     ab = signFMod(ab + M_PI,(M_PI * 2)) - M_PI;
//     double ba = (b - a);
//     ba = signFMod(ba + M_PI,(M_PI * 2)) - M_PI;
// //     printf("ab %f, ba %f\n", ab, ba);
//     double result = 0;
//     if(fabs(ba) >= fabs(ab)) {
//         result  = ab;
//     } else {
//         result = ba;
//     }
//     return result;
//     printf("a: %f b: %f\nf: %f\n",a ,b, b-a);
    return b-a;
}

JointSpeedBasedPositionController::JointSpeedBasedPositionController(double radiusIn, int minPWMIn, int maxPWMIn, int maxRPMIn, char * topic, ros::NodeHandle nh, std::string name, bool flipDirection) :
    JointController(topic,nh,name),
    flipDirection(flipDirection) {
        
    radius = radiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    turnsCorrectingFor = 0;
    deltaPWM = maxPWMIn - minPWMIn;
    maxVelocity = (maxRPMIn * 2.0*M_PI/SECONDS_IN_MINUTE) ;
    velocityRange = maxVelocity * 2.0;
    printf("deltaPWM %d, maxVelocity %f velocityRange %f\n", deltaPWM, maxVelocity, velocityRange);
    pwmVelRatio = (deltaPWM) / (velocityRange);
    
    
    //inital values
    lastAimPosition     = std::numeric_limits< double >::quiet_NaN();
    lastKnownPosition   = std::numeric_limits< double >::quiet_NaN();
    lastAngularVelocity = std::numeric_limits< double >::infinity();
    
    
    lastPWM = minPWMIn + deltaPWM;
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
    
    double targetAngularVel;
    double deltaT = 1.0/updateFrequency;
    int pwm;
    
    //Step 1: Calculate the angular velocity currently
    double currentAngVel = 0.0;
    double posDelta = 0.0;
    if(!std::isnan(lastKnownPosition)) {
        posDelta = calcShortestCircDelta(currentPos, lastKnownPosition);
    }
    currentAngVel = posDelta/deltaT;
    printf("posDelta %f, deltaT %f, angVel %f\n", posDelta, deltaT, currentAngVel);


    //check for infinity - we should try to return to the center
    if(std::isinf<double>(currentPos)) {
        ROS_ERROR("Current Position is infinity!");

        //if we've been trying for more than 1 second give up
        if(turnsCorrectingFor > updateFrequency) {
            targetAngularVel = 0.0;
            ROS_ERROR("Tried to return for too long, giving up");
        } else if (currentPos > 0) {
            targetAngularVel = -VEL_INC;
        } else { //currentPos < 0
            targetAngularVel = VEL_INC;
        }
        
    //check for invalid values
    } else if ( std::fabs(futurePos) + FLOATING_PT_ERROR > (M_PI * 2)) {
        ROS_ERROR("angle %f radians is not a valid position. Positions should be btween %f and %f",futurePos,(M_PI * 2),-(M_PI * 2));
        targetAngularVel = 0.0;
    } else {
        //reset our limit correction count to zero
        turnsCorrectingFor = 0;
        
        //place the two angles in our range
        futurePos = posRangeConvert(futurePos);
        currentPos = posRangeConvert(currentPos);
        printf("future %f, current %f, update %f\n", futurePos, currentPos, updateFrequency);
        
        double radialDistToTarget =  calcShortestCircDelta(futurePos, currentPos);
        
        //escape if we are close enough
        if(fabs(radialDistToTarget) < (0.2)) {
            lastKnownPosition = currentPos;
            lastAimPosition = futurePos;
            lastDeltaT = deltaT;
            printf("close enough\n");
            
            targetAngularVel = 0.0;
        } else {
        
            targetAngularVel = currentAngVel;
            
            //Step 2: Calculate the direction to reach the desired position fastest (CW, CCW)
            
            
            //are we at zero?
            //if(fabs(currentAngVel) > VEL_ERROR) {
              /*  printf("maxVel: %f, currentAngVel: %f\n", maxVelocity, currentAngVel);
                printf("non zero vel\n");
                //Are we going in the right direction   
                if(std::signbit(radialDistToTarget) == std::signbit(currentAngVel)) { //compares the sign of the two
                    printf("sign equal\n");
                    if(targetAngularVel > 0) {
                        targetAngularVel = std::min(currentAngVel+VEL_INC,maxVelocity);
                    } else if (targetAngularVel < 0) {
                        targetAngularVel = std::max(currentAngVel-VEL_INC,-maxVelocity);
                    }
                } else {
                    printf("sign not equal\n");
                    //this will reverse the sign if it needs to be negative
                    if(currentAngVel > 0) {
                        targetAngularVel = VEL_INC;   
                    } else {
                        targetAngularVel = -VEL_INC;
                    }
                    
                }
                printf("targetAngularVel %f\n", targetAngularVel);
            /*} else {
                printf("zero vel\n");
                
                if(radialDistToTarget >  SMALL_INC) {
                    targetAngularVel = VEL_INC;
                } else if (radialDistToTarget < -SMALL_INC) {
                    targetAngularVel = -VEL_INC;
                } else {
                    printf("stop");
                    targetAngularVel = 0;
                }
                
            }*/

            
            //Step 3: will we pass the point this turn
            double nextPosGuess = posRangeConvert(targetAngularVel * updateFrequency);
            
            double nextPosDelta = calcShortestCircDelta(currentPos, nextPosGuess);
            printf("nextPosGuess %f nextPosDelta %f\n", nextPosGuess, nextPosDelta);
            //step back the speed to a more sensible one if we are going to miss the point
//             if(std::fabs(nextPosDelta) > std::fabs(radialDistToTarget)) {
                //decrease to a value that will get us just before it
                targetAngularVel = std::max(-maxVelocity, std::min(maxVelocity, (radialDistToTarget * deltaT)));
                nextPosGuess = posRangeConvert(targetAngularVel * updateFrequency);
                nextPosDelta = calcShortestCircDelta(nextPosGuess, currentPos);
            //}
            nextPosGuess = (targetAngularVel * updateFrequency);
            
            
        }
    }
    if(flipDirection) {
        targetAngularVel*=-1.0;
    }
    
    pwm = (int)(targetAngularVel * pwmVelRatio) + minPWM + (deltaPWM/2);
    printf("maxVel %f, aimVel %f, currentVel %f\n", maxVelocity, targetAngularVel, currentAngVel);
//     printf("pwm %d, gearAdjustedVel %f, multi %f\n", pwm, gearAdjustedVel, gearAdjustedVel * pwmVelRatio);
    
    #ifdef DO_VEL_ADJUST
        if(lastAngularVelocity !=  std::numeric_limits< double >::infinity() &&
            !FLOAT_EQL(lastAngularVelocity,lastTargetVelocity)) {
            int error = pwmVelRatio * (lastAngularVelocity - lastTargetVelocity) * gearMultiplier;
            pwm += error;
        }
    #endif
    lastPWM = pwm;
    lastTargetVelocity = targetAngularVel;
    lastKnownPosition = currentPos;
    lastAimPosition = futurePos;
    lastAngularVelocity = currentAngVel;
    lastDeltaT = deltaT;
    printf("pwm %d\n", pwm);
    return pwm;
}

int JointSpeedBasedPositionController::posToPWM ( double currentPos, double updateFrequency ) {
    printf("requested %f\n", requestedValue);
    return posToPWM(requestedValue, currentPos, updateFrequency);
}


jointInfo JointSpeedBasedPositionController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime) {
    ros::Duration dif = extrapolationTime - sessionStart;
    double aimVel = calcShortestCircDelta(lastKnownPosition, lastAimPosition)/lastDeltaT; //NOTE: as we are accelerating/deacclerating this is probably wrong
    
    jointInfo info;
    info.position = fmod(aimVel * dif.toSec() + lastKnownPosition, (M_PI * 2));
//     info.position = lastKnownPosition;
    info.velocity = aimVel; //TODO: extrapolate based on delta
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}

    
