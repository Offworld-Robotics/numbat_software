/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */

#include "LIDARTiltJointController.hpp"
#include <math.h>
#include <limits>

#define LIDAR_HORIZ 1330.0
#define LIDAR_FREQ 5.0
#define DEG_PER_PWM 0.12328767123
#define SECONDS_DELAY 0.02876707662
#define PWM_SHIFT 10.0

#define FORWARDS 0
#define BACKWARDS 1

//if this is not defined do it just based on pwm
// #define DO_VEL_ADJUST

static double pwmToPos(double pwm) {
    double lidarRads;
    lidarRads = pwm - LIDAR_HORIZ;
    lidarRads = lidarRads * DEG_PER_PWM;//Find angle in degrees
    lidarRads = lidarRads * M_PI/180.0;//Convert to radians
    return lidarRads;
}


LIDARTiltJointController::LIDARTiltJointController(LIDARTiltMode aMode, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
    //initalise mode subscriber
    modeSub = nh.subscribe("/owr/lidar_gimble_mode", 10, &LIDARTiltJointController::setModeCallback, this);
    
    angle = LIDAR_HORIZ;
    currentDirection = FORWARDS;
    mode = aMode;
    //inital values
    lastAngularVelocity = std::numeric_limits< double >::infinity();
    
    lastPWM = LIDAR_HORIZ;
    lastHz = LIDAR_FREQ;
}

/*
 * See notes at https://bluesat.atlassian.net/browse/OWRS-203 for maths explanation
 * Inputs:
 *        - targetVel  - the target point velocity of the wheel
 *        - currentVel - the current point velocity of the wheel 
 *        - currentAngle - the angle of the rover to the ground (incline)
 * 
 */
int LIDARTiltJointController::velToPWM(double requestedValue, double hz) {
    int pwm;
    printf("mode %d\n", mode);
    if(mode == CONTINUOUS) {
        //Test constraints for osciallting lidar. Ignores function inputs
        if(angle >= 1700){
            currentDirection = BACKWARDS; // change direction at 45 degrees from oriz.
            angle -= PWM_SHIFT;
        } else if (angle <= 960){
            currentDirection = FORWARDS; //change direction at 45 degrees from horiz.
            angle += PWM_SHIFT;
        } else {
            angle += PWM_SHIFT * (1 - (2 * currentDirection)); // equivalent of, if(forward), increment, if(backward) decrement
        }
        lastAngularVelocity = (PWM_SHIFT * hz) * DEG_PER_PWM * (M_PI/180);
        
        pwm = angle;
        printf("pwm %d\n", pwm);
    } else if (mode == STATIONARY) {
        pwm = LIDAR_HORIZ;
        lastAngularVelocity = 0;
    } else { //position
        pwm = (requestedValue * 180.0) / (DEG_PER_PWM * M_PI) + LIDAR_HORIZ;
        lastAngularVelocity  = (((lastPWM - LIDAR_HORIZ) * DEG_PER_PWM * (M_PI/180.0)) - requestedValue) * hz;
	ROS_INFO("pwm %d requested %f", pwm, requestedValue);
    }
    lastHz = hz;
    lastPWM = pwm;
//     printf("pwm %d\n", pwm);
    return pwm;
}


int LIDARTiltJointController::velToPWM (double hz ) {
    //safe value
    if (isnan(requestedValue) && mode == POSITION) {
            return lastPWM;
    }
    return velToPWM(requestedValue, hz);
}

void LIDARTiltJointController::setModeCallback(const std_msgs::Int16 modeMsg) {
    mode = (LIDARTiltMode) modeMsg.data;
    ROS_INFO("Mode %d",(int) mode);
}


// Sends rviz an angle (radians) representing the current position of the lidar
// gimbal. Angle is measured from horizontal (1330:pwm = 0 rads), with tilting
// towards the front of the rover measured as positive radians, and tilting in
// the opposite direction as negative.
jointInfo LIDARTiltJointController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime) {
    
    double lidarRads;
    double lidarVel;
    lidarRads = pwmToPos(lastPWM);
    
    //NO extrapolation for position or stationary modes yet
    if(mode == POSITION) {
        lidarVel = (PWM_SHIFT * lastHz) * DEG_PER_PWM * (M_PI/180); // find velocity in rad/s
        
        if(currentDirection == BACKWARDS){
            lidarVel = lidarVel * (-1);
        }
    
    } else if (mode == STATIONARY) {
        lidarVel = 0.0;
    } else { //CONTINUOUS
        ros::Time atPosition = sessionStart + ros::Duration(SECONDS_DELAY);
        if(atPosition >= extrapolationTime) {
            //DO NOTHING, should stop at this point
            lidarVel = 0;
        } else if (atPosition > extrapolationTime) {
            lidarVel = (PWM_SHIFT * lastHz) * DEG_PER_PWM * (M_PI/180); // find velocity in rad/s
            float secondsToPoint = (atPosition - extrapolationTime).toSec();
            //NOTE: this assumes that we did not start closer to the point then the full time it takes us to reach it.
            lidarRads -= lidarRads - lidarVel * secondsToPoint;
        }
        
        
        if(currentDirection == BACKWARDS){
            lidarVel = lidarVel * (-1);
        }
    }
    
    jointInfo info;
    info.position = lidarRads;
    
    info.velocity = lidarVel; 
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}

