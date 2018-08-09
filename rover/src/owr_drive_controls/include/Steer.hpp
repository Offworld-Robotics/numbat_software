/*
 * Orignal Author: Sajid Ibne Anower
 * Editors: Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Defines the data structure used to do steering
 */

#include <geometry_msgs/Twist.h>

#ifndef SRC_OWR_DRIVE_CONTROLS_INCLUDE_STEER_HPP
#define STEER_H

typedef struct MotorVels {
    double frontLeftMotorV,
           frontRightMotorV,
           backLeftMotorV,
           backRightMotorV;

    double frontLeftAng,
           frontRightAng,
           backRightAng,
           backLeftAng;
} MotorVels;

MotorVels doTranslation(const geometry_msgs::Twist * velMsg);

#endif  // SRC_OWR_DRIVE_CONTROLS_INCLUDE_STEER_HPP
