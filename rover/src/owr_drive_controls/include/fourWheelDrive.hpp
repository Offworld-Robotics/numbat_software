/**
 * @authors: (Orignal Author) Anita Smirnov, Alan Ngyuen
 * Editors: 
 * Date Started: 3/04/2018
 * @description:: Defines the data structure used to do steering
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017
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
