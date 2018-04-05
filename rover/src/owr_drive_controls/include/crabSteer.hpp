/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 19/02/2016
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
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

#endif  // SRC_OWR_DRIVE_CONTROLS_INCLUDE_CRABSTEER_HPP_
