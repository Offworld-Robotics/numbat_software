/*
 * Defines the data structure used to do crab steering
 */

#include <geometry_msgs/Twist.h>

#ifndef SRC_OWR_DRIVE_CONTROLS_INCLUDE_CRABSTEER_HPP
#define CRAB_STEER_H

typedef struct crabMotorVels {
    double frontLeftMotorV,
           frontRightMotorV,
           backLeftMotorV,
           backRightMotorV;

    double frontLeftAng,
           frontRightAng,
           backRightAng,
           backLeftAng;
} crabMotorVels;

crabMotorVels doCrabTranslation(const geometry_msgs::Twist * velMsg);

#endif  // SRC_OWR_DRIVE_CONTROLS_INCLUDE_CRABSTEER_HPP
