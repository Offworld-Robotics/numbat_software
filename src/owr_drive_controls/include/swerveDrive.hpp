/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 19/02/2016
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 */

#ifndef SWERVE_DRIVE_H
#define SWERVE_DRIVE_H

#include "Drive.hpp"

class SwerveDrive: public Drive {
    public:
        motorVels doVelTranslation(const geometry_msgs::Twist * velMsg);
    private:    
        motorVels steer(motorVels vels, double velMagnitude, double turnAngle);
        int direction;
};

#endif  // SWERVE_DRIVE_H
