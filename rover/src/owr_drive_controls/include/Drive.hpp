/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 */
/*
 * Date Started: 19/02/2016
 * Orignal Author: Harry J.E Day
 * Editors: Alan Nyguen
 * Purpose: Represents an interface for translating a given velocity vector to wheel vectors
 * This code is released under the MIT License. Copyright BLUEsat UNSW, 2018
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <geometry_msgs/Twist.h>

typedef struct _motorVels {
    double frontLeftMotorV,
           frontRightMotorV,
           backLeftMotorV,
           backRightMotorV;

    double frontLeftAng,
           frontRightAng,
           backRightAng,
           backLeftAng;
} motorVels;

class Drive {
    public:
        virtual motorVels doVelTranslation(const geometry_msgs::Twist * velMsg) = 0;
    protected:
        virtual motorVels steer(motorVels vels, double velMagnitude, double turnAngle) = 0;
        motorVels stop(motorVels vels); 
        double getVelMagnitude(const geometry_msgs::Twist * velMsg);
        double getHypotenuse(double x, double y);
        int getDir(double);
};

#endif //DRIVE_H
