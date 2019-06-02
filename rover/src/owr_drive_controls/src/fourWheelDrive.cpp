/*
 * @author: (Orignal Author) Sajid Ibne Anower
 * @authors: (Editors) Anita Smirnov, Kevin Chan
 * Date Started: 3/04/2018
 * Purpose: Implementation of four wheel steering logic
 * @copyright: This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017
 */


#include "fourWheelDrive.hpp"
#include <cmath>
#include <ros/ros.h>

#define VEL_ERROR 0.01

#define FRONT 1
#define BACK -FRONT

#define FWDRIVE_LIMIT 0.45*M_PI/2

/*
 * @arg1 vels: current motor info
 * @arg2 velMag: 
 * @arg3 turnAng
 *
 * @return output: vels translated
 *
 * Desciption: translate the current vels with the new turnAngle
 */
motorVels FourWheelDrive::steer(motorVels vels, double velMagnitude, double turnAngle) {
    motorVels output = vels;
    output.frontLeftMotorV = velMagnitude;
    output.backLeftMotorV = velMagnitude;
    output.frontRightMotorV = velMagnitude;
    output.backRightMotorV = velMagnitude;

    // Capping the turnangle to 45% of M_PI/2
    if(turnAngle >= FWDRIVE_LIMIT) {
        turnAngle = FWDRIVE_LIMIT;
    } else if(turnAngle <= -FWDRIVE_LIMIT) {
        turnAngle = -FWDRIVE_LIMIT;
    }

    output.frontLeftAng = output.frontRightAng = turnAngle;
    output.backLeftAng = output.backRightAng = -turnAngle;
    return output;
    
	// work out the center of the circle and the radius of the circle
			// Primary Circle
				// radius: (cos(90 - turnAngle) * rover_height)/2
				// Assume origin at the center of the rover
				// center: at ((rover_height*tan(90-turnAngle))/2, 0)

			// Secondary Circle
				// using pythag
				// radius^2 = (rover_width+center)^2 - (rover_height/2)^2
				// center: same as primary

			// Turn angles
				// wheels on the primary circle
					// front wheel: turnAngle
					// back wheel: -turnAngle
				// wheels on the secondary cicle
					// tan(90 - fi) = (rover_width + center)/(rover_height/2)
					// front wheel: fi
				    // back wheel: -fi

		// if it is a right turn, center offset is +ve
		// if it is a left turn, center offset is -ve
}


motorVels FourWheelDrive::doVelTranslation(const geometry_msgs::Twist * velMsg) {
    motorVels output;
    double velMagnitude = getVelMagnitude(velMsg);
    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, fabs(velMsg->linear.x));
        int dir = getDir(velMsg->linear.x);
        // (turnAngle * dir) is the final normalised angle,
        // required to make the driving similar to that of a car
        double normalisedAngle = turnAngle * dir;
        output = steer(output, dir * velMagnitude, normalisedAngle);
    } else {
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
