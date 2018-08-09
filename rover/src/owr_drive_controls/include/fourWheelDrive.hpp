/*
 * Orignal Author: Sajid Ibne Anower
 * Editors: Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Defines the data structure used to do steering
 */

#ifndef FOUR_WHEEL_DRIVE_H
#define FOUR_WHEEL_DRIVE_H

#include "Drive.hpp"

class FourWheelDrive: public Drive {
    public:
        motorVels doVelTranslation(const geometry_msgs::Twist * velMsg);
    private:    
        motorVels steer(motorVels vels, double velMagnitude, double turnAngle);
};

#endif  // FOUR_WHEEL_DRIVE_H
