/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 19/02/2016
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 */

#include <geometry_msgs/Twist.h>

#ifndef SWERVE_DRIVE_H
#define SWERVE_DRIVE_H

typedef struct _swerveMotorVels {
    double frontLeftMotorV,
           frontRightMotorV,
           backLeftMotorV,
           backRightMotorV;

    double frontLeftAng,
           frontRightAng;
} swerveMotorVels;

swerveMotorVels doVelTranslation ( const geometry_msgs::Twist * velMsg );

#endif //SWERVE_DRIVE_H
