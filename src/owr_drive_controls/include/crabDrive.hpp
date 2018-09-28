/**
 * @author: (Orignal Author) Sajid Ibne Anower
 * @authors: (editors) Anita Smirnov
 * Date Started: 3/04/2018
 * Purpose: Defines the data structure used to do steering
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2018
 */

#ifndef CRAB_DRIVE_H
#define CRAB_DRIVE_H

#include "Drive.hpp"

class CrabDrive: public Drive {
    public:
        motorVels doVelTranslation(const geometry_msgs::Twist * velMsg);
    private:    
        motorVels steer(motorVels vels, double velMagnitude, double turnAngle);
};

#endif  // CRAB_DRIVE_H
