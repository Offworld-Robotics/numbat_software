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

//if this is not defined do it just based on pwm
// #define DO_VEL_ADJUST

static inline double signFMod(double v, double mod) {
    return v - floor(v/mod) * mod;
}

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
    
    double ab = (a - b);
    ab = signFMod(ab + M_PI,(M_PI * 2)) - M_PI;
    double ba = (b - a);
    ba = signFMod(ba + M_PI,(M_PI * 2)) - M_PI;
    printf("ab %f, ba %f\n", ab, ba);
    double result = 0;
    if(fabs(ba) >= fabs(ab)) {
        result  = ab;
    } else {
        result = ba;
    }
    return result;
}

JointSpeedBasedPositionController::JointSpeedBasedPositionController(double radiusIn, const double * gearRatioIn, int nGearsIn,int minPWMIn, int maxPWMIn, int maxRPMIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
    radius = radiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    deltaPWM = maxPWMIn - minPWMIn;
    maxVelocity = (maxRPMIn * M_2_PI/SECONDS_IN_MINUTE) ;
    velocityRange = maxVelocity * 2.0;
    printf("deltaPWM %d, maxVelocity %f velocityRange %f\n", deltaPWM, maxVelocity, velocityRange);
    
    
    //inital values
    lastAimPosition     = std::numeric_limits< double >::quiet_NaN();
    lastKnownPosition   = std::numeric_limits< double >::quiet_NaN();
    lastAngularVelocity = std::numeric_limits< double >::infinity();
    
    gearRatio = gearRatioIn;
    nGears = nGearsIn;
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
    printf("future %f, current %f, update %f\n", futurePos, currentPos, updateFrequency);
    //check for invalid values
    if( std::fabs(futurePos) + FLOATING_PT_ERROR > (M_PI * 2)) {
        ROS_ERROR("angle %f radians is not a valid position. Positions should be btween %f and %f",futurePos,(M_PI * 2),-(M_PI * 2));
        printf("angle %f radians is not a valid position. Positions should be btween %f and %f\n",futurePos,(M_PI * 2),-(M_PI * 2));
        return -1;
    }
    
    double deltaT = 1.0/updateFrequency;
    
    //Step 1: Calculate the angular velocity currently
    double currentAngVel = 0.0;
    double posDelta = futurePos;
    if(!std::isnan(lastKnownPosition)) {
         posDelta = calcShortestCircDelta(currentPos, lastKnownPosition);
    }
    currentAngVel = posDelta/deltaT;
    double targetAngularVel = currentAngVel;
    //Step 2: Calculate the direction to reach the desired position fastest (CW, CCW)
    double radialDistToTarget =  calcShortestCircDelta(futurePos, currentPos);
    
    printf("updateFrequency %f, deltaT %f, currentAngVel %f, targetAngularVel %f, radialDistToTarget %f\n", updateFrequency, deltaT, currentAngVel, targetAngularVel, radialDistToTarget);
    
    //TODO: check limits and efficiencies here
    
    //are we at zero?
    if(fabs(currentAngVel) > VEL_INC) {
        //Are we going in the right direction
        if(std::signbit(radialDistToTarget) == std::signbit(currentAngVel)) { //compares the sign of the two
            if(targetAngularVel > 0) {
                targetAngularVel = std::max(currentAngVel+VEL_INC,maxVelocity);
            } else if (targetAngularVel < 0) {
                targetAngularVel = std::min(currentAngVel-VEL_INC,-maxVelocity);
            }
        } else {
            //this will reverse the sign if it needs to be negative
            if(currentAngVel > 0) {
                targetAngularVel = maxVelocity;   
            } else {
                targetAngularVel = -maxVelocity;
            }
            
        }
    } else {
        if(radialDistToTarget > 0 + SMALL_INC) {
            targetAngularVel = VEL_INC;
        } else if (radialDistToTarget < 0 -  SMALL_INC) {
            targetAngularVel = -VEL_INC;
        }
        //NOTE: if radialDistToTarget == 0 we don't want to increase_
    }
    printf("targetAngularVel %f\n", targetAngularVel);
    
    //Step 3: will we pass the point this turn
    double nextPosGuess = std::fmod((targetAngularVel * updateFrequency), (M_PI * 2));
    
    double nextPosDelta = calcShortestCircDelta(nextPosGuess, currentPos);
    printf("nextPosGuess %f nextPosDelta %f\n", nextPosGuess, nextPosDelta);
//     if(std::fabs(nextPosDelta) >= std::fabs(posDelta)) {
//         //decrease to a value that will get us just before it
//         targetAngularVel = (((futurePos - SMALL_INC) - currentPos) / deltaT);
//     }
    //TODO: correctly implement the above
    nextPosGuess = (targetAngularVel * updateFrequency);
    if(std::fabs(nextPosGuess) > (M_PI * 2) ) {
        
        //TODO: this is a danger
    }
    
    
    lastTargetVelocity = targetAngularVel;
    
    
    
    double pwmVelRatio = (deltaPWM) / (velocityRange);
    printf("pwmVelRatio %f, targetAngularVel %f\n", pwmVelRatio, targetAngularVel);
    double gearAdjustedVel = targetAngularVel;
    double gearMultiplier = 1;
    int i;
    for(i = 0; i < nGears; i++) {
        gearMultiplier *= gearRatio[i];
    }
    gearAdjustedVel *= gearMultiplier;
    printf("gearAdjustedVel %f\n", gearAdjustedVel);
    if (gearAdjustedVel > 0.0) {
        gearAdjustedVel = std::min<double>(gearAdjustedVel, maxVelocity);
        printf("gearAdjustedVel max %f\n", gearAdjustedVel);
    } else {
        gearAdjustedVel = std::max<double>(gearAdjustedVel, -maxVelocity);
         printf("gearAdjustedVel min %f\n", gearAdjustedVel);
    }
    int pwm = (int)(gearAdjustedVel * pwmVelRatio) + minPWM + (deltaPWM/2);
    printf("pwm %d, gearAdjustedVel %f, multi %f\n", pwm, gearAdjustedVel, gearAdjustedVel * pwmVelRatio);
    
    #ifdef DO_VEL_ADJUST
        if(lastAngularVelocity !=  std::numeric_limits< double >::infinity() &&
            !FLOAT_EQL(lastAngularVelocity,lastTargetVelocity)) {
            int error = pwmVelRatio * (lastAngularVelocity - lastTargetVelocity) * gearMultiplier;
            pwm += error;
        }
    #endif
    
    lastPWM = pwm;
    lastKnownPosition = currentPos;
    lastAimPosition = futurePos;
    lastAngularVelocity = currentAngVel;
    lastDeltaT = deltaT;
    
    return pwm;
}

int JointSpeedBasedPositionController::posToPWM ( double currentPos, double updateFrequency ) {
    return posToPWM(requestedValue, currentPos, updateFrequency);
}


jointInfo JointSpeedBasedPositionController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime) {
    ros::Duration dif = extrapolationTime - sessionStart;
    double aimVel = calcShortestCircDelta(lastKnownPosition, lastAimPosition)/lastDeltaT; //NOTE: as we are accelerating/deacclerating this is probably wrong
    
    jointInfo info;
    info.position = fmod(aimVel * dif.toSec() + lastKnownPosition, (M_PI * 2));
    info.velocity = aimVel; //TODO: extrapolate based on delta
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}
    
