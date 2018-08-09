/*
 * Orignal Author: Sajid Ibne Anower
 * Editors: Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Defines the data structure used to do steering
 */

#include <geometry_msgs/Twist.h>

#ifndef SRC_OWR_DRIVE_CONTROLS_INCLUDE_FOURWHEELSTEER_HPP
#define FOUR_WHEEL_STEER_H

typedef struct fourWheelMotorVels {
    double frontLeftMotorV,
           frontRightMotorV,
           backLeftMotorV,
           backRightMotorV;

    double frontLeftAng,
           frontRightAng,
           backRightAng,
           backLeftAng;
} fourWheelMotorVels;

fourWheelMotorVels doFourWheelTranslation(const geometry_msgs::Twist * velMsg);

#endif  // SRC_OWR_DRIVE_CONTROLS_INCLUDE_FOURWHEELSTEER_HPP
