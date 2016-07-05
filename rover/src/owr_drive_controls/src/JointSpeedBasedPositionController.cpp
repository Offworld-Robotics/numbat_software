/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointSpeedBasedPositionController.hpp"
#include <math.h>
#include <ros/ros.h>

#define VEL_INC 10.2
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

JointSpeedBasedPositionController::JointSpeedBasedPositionController(double radiusIn, const double * gearRatioIn, int nGearsIn,int minPWMIn, int maxPWMIn, int maxRPMIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
    radius = radiusIn;
    maxPWM = maxPWMIn;
    minPWM = minPWMIn;
    maxRPM = maxRPMIn;
    deltaPWM = maxPWMIn - minPWMIn;
    maxVelocity = (maxRPMIn * 2*M_PI/SECONDS_IN_MINUTE) ;
    velocityRange = maxVelocity * 2.0;
    printf("deltaPWM %d, maxVelocity %f velocityRange %f\n", deltaPWM, maxVelocity, velocityRange);
    
    
    //inital values
    lastAimPosition     = std::numeric_limits< double >::quiet_NaN();
    lastKnownPosition   = std::numeric_limits< double >::quiet_NaN();
    lastAngularVelocity = std::numeric_limits< double >::infinity();
    
    gearRatio = gearRatioIn;
    nGears = nGearsIn;
    
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
    
    //check for infinity - we should stop
    if(std::isinf<double>(currentPos)) {
        ROS_ERROR("Current Position is infinity!");
        return deltaPWM/2 + minPWM;
    }
    
    //check for invalid values
    if( std::fabs(futurePos) + FLOATING_PT_ERROR > (M_PI * 2)) {
        ROS_ERROR("angle %f radians is not a valid position. Positions should be btween %f and %f",futurePos,(M_PI * 2),-(M_PI * 2));
        printf("angle %f radians is not a valid position. Positions should be btween %f and %f\n",futurePos,(M_PI * 2),-(M_PI * 2));
        return -1;
    }
    
    //place the two angles in our range
    futurePos = posRangeConvert(futurePos);
    currentPos = posRangeConvert(currentPos);
    printf("future %f, current %f, update %f\n", futurePos, currentPos, updateFrequency);
    
    double deltaT = 1.0/updateFrequency;
    double aimPosDelta = calcShortestCircDelta(currentPos, futurePos);
    
    //escape if we are close enough
    if(fabs(aimPosDelta) < (0.15)) {
        int pwm = deltaPWM/2 + minPWM; 
        printf("mid pwm %d, posDelta %f\n", pwm, aimPosDelta);
//         nextPosGuess = currentPos;
        lastPWM = pwm;
        lastKnownPosition = currentPos;
        lastAimPosition = futurePos;
        lastAngularVelocity = 0.0;
        lastDeltaT = deltaT;
        printf("close enoug\n");
        return pwm;
    }
    
    //Step 1: Calculate the angular velocity currently
    double gearMultiplier = 1;
    int i;
    for(i = 0; i < nGears; i++) {
        gearMultiplier /= gearRatio[i];
    }
    double currentAngVel = 0.0;
    double posDelta = futurePos;
    if(!std::isnan(lastKnownPosition)) {
         posDelta = calcShortestCircDelta(currentPos, lastKnownPosition);
    }
    currentAngVel = posDelta/deltaT;
    currentAngVel = currentAngVel * gearMultiplier;
    double targetAngularVel = currentAngVel;
    //Step 2: Calculate the direction to reach the desired position fastest (CW, CCW)
    double radialDistToTarget =  calcShortestCircDelta(futurePos, currentPos);
    
//     printf("updateFrequency %f, deltaT %f, currentAngVel %f, targetAngularVel %f, radialDistToTarget %f\n", updateFrequency, deltaT, currentAngVel, targetAngularVel, radialDistToTarget);
    
    //TODO: check limits and efficiencies here
    
    //are we at zero?
    if(fabs(currentAngVel) > VEL_ERROR) {
        printf("non zero vel\n");
        //Are we going in the right direction   
        if(std::signbit(radialDistToTarget) == std::signbit(currentAngVel)) { //compares the sign of the two
            printf("sign not equal\n");
            if(targetAngularVel > 0) {
                targetAngularVel = std::min(currentAngVel+VEL_INC,maxVelocity);
            } else if (targetAngularVel < 0) {
                targetAngularVel = std::max(currentAngVel-VEL_INC,-maxVelocity);
            }
        } else {
            printf("sign equal\n");
            //this will reverse the sign if it needs to be negative
            if(currentAngVel > 0) {
                targetAngularVel = VEL_INC;   
            } else {
                targetAngularVel = -VEL_INC;
            }
            
        }
    } else {
        printf("zero vel\n");
        
        if(radialDistToTarget > 0 + SMALL_INC) {
            targetAngularVel = VEL_INC;
        } else if (radialDistToTarget < 0 -  SMALL_INC) {
            targetAngularVel = -VEL_INC;
        }
        //NOTE: if radialDistToTarget == 0 we don't want to increase_
    }

    
    //Step 3: will we pass the point this turn
    double nextPosGuess = std::fmod((targetAngularVel * updateFrequency), (M_PI * 2));
    
    double nextPosDelta = calcShortestCircDelta(nextPosGuess, currentPos);
    printf("nextPosGuess %f nextPosDelta %f\n", nextPosGuess, nextPosDelta);
    //step back the speed to a more sensible one if we are going to miss the point
    if(std::fabs(nextPosDelta) > std::fabs(posDelta)) {
        //decrease to a value that will get us just before it
        targetAngularVel = (calcShortestCircDelta((futurePos - SMALL_INC) , currentPos) / deltaT);
        nextPosGuess = std::fmod((targetAngularVel * updateFrequency), (M_PI * 2));
        nextPosDelta = calcShortestCircDelta(nextPosGuess, currentPos);
    }
    //TODO: correctly implement the above
    nextPosGuess = (targetAngularVel * updateFrequency);
//     if(std::fabs(nextPosGuess) > (M_PI * 2) ) {
//         
//         //TODO: this is a danger
//     }
    
    
    lastTargetVelocity = targetAngularVel;
    
//     printf(" targetAngularVel %f\n",  targetAngularVel);
    double gearAdjustedVel = targetAngularVel;
    
    gearAdjustedVel *= gearMultiplier;
//     printf("gearAdjustedVel %f\n", gearAdjustedVel);
    if (gearAdjustedVel > 0.0) {
        gearAdjustedVel = std::min<double>(gearAdjustedVel, maxVelocity);
//         printf("gearAdjustedVel max %f\n", gearAdjustedVel);
    } else {
        gearAdjustedVel = std::max<double>(gearAdjustedVel, -maxVelocity);
//          printf("gearAdjustedVel min %f\n", gearAdjustedVel);
    }
    
    
    double pwmVelRatio = (deltaPWM) / (velocityRange);
    int pwm = (int)(gearAdjustedVel * pwmVelRatio) + minPWM + (deltaPWM/2);
    printf("maxVel %f, aimVel %f, currentVel %f\n", maxVelocity, gearAdjustedVel, currentAngVel);
//     printf("pwm %d, gearAdjustedVel %f, multi %f\n", pwm, gearAdjustedVel, gearAdjustedVel * pwmVelRatio);
    
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
    
