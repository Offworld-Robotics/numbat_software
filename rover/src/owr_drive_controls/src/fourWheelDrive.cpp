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

#define DEG180 M_PI
#define DEG90 M_PI_2

#define FWDRIVE_LIMIT 0.45*M_PI/2
#define ROVER_HEIGHT .47// distance between front and back wheel on the same side in m
#define ROVER_WIDTH .76// distance between two front wheels in m

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
    //motorVels output = vels;
    //output.frontLeftMotorV = velMagnitude;
    //output.backLeftMotorV = velMagnitude;
    //output.frontRightMotorV = velMagnitude;
    //output.backRightMotorV = velMagnitude;

    // Capping the turnangle to 45% of M_PI/2
    if(turnAngle >= FWDRIVE_LIMIT) {
        turnAngle = FWDRIVE_LIMIT;
    } else if(turnAngle <= -FWDRIVE_LIMIT) {
        turnAngle = -FWDRIVE_LIMIT;
    }

	// Primary Circle
		// radius: (cos(90 - turnAngle) * ROVER_HEIGHT)/2
		// Assume origin at the center of the rover
		// center: at ((ROVER_HEIGHT*tan(90-turnAngle))/2, 0)

	// Secondary Circle
		// using pythag
		// radius^2 = (ROVER_WIDTH+center)^2 + (ROVER_HEIGHT/2)^2
		// center: same as primary

	// Turn angles
		// wheels on the primary circle
			// front wheel: turnAngle
			// back wheel: -turnAngle
		// wheels on the secondary cicle
			// tan(90 - fi) = (ROVER_WIDTH + center)/(ROVER_HEIGHT/2)
			// front wheel: fi
			// back wheel: -fi

    motorVels output = vels; 
    double center = (ROVER_HEIGHT*tan(90-turnAngle))/2;
    double primaryRadius = (cos(DEG90 - turnAngle) * ROVER_HEIGHT)/2;
    double angularVelocity = velMagnitude/primaryRadius;
    double secondaryRadius = sqrt( pow(ROVER_WIDTH+center, 2) + pow(ROVER_HEIGHT/2, 2));
    if(0 < turnAngle) {
	    ROS_INFO("Right turn");
	    // Turn Angles
	    output.frontRightAng = turnAngle;
	    output.backRightAng = -turnAngle;
	    output.frontLeftAng = DEG90 - atan2(ROVER_WIDTH + center, ROVER_HEIGHT/2);
	    output.backLeftAng = -output.frontLeftAng;

	    // velocities:
	    output.frontLeftMotorV = angularVelocity;
	    output.backLeftMotorV = angularVelocity;

	    angularVelocity = velMagnitude/secondaryRadius;
	    output.frontRightMotorV = angularVelocity;
	    output.backRightMotorV = angularVelocity;
    } else if(turnAngle > M_PI) {
	    ROS_INFO("Left turn");
	    output.frontLeftAng = turnAngle;
	    output.backLeftAng = -turnAngle;
	    output.frontRightAng = DEG90 - atan2(ROVER_WIDTH + center, ROVER_HEIGHT/2);
	    output.backRightAng = -output.frontRightAng;

	    // velocities:
	    output.frontRightMotorV = angularVelocity;
	    output.backRightMotorV = angularVelocity;

	    angularVelocity = velMagnitude/secondaryRadius;
	    output.frontLeftMotorV = angularVelocity;
	    output.backLeftMotorV = angularVelocity;
    } else {
	    // doVelTranslation takes care of straight pathing i think...
	    ROS_INFO("turn angle unaccounted for");
    }
	
    return output;
}


motorVels FourWheelDrive::doVelTranslation(const geometry_msgs::Twist * velMsg) {
    motorVels output;
    double velMagnitude = getVelMagnitude(velMsg);
    // If the magnitude is close to zero
    if (velMagnitude < VEL_ERROR) {
        output = stop(output);
    } else if (fabs(velMsg->linear.y) >= VEL_ERROR) {
        const double turnAngle = atan2(velMsg->linear.y, fabs(velMsg->linear.x));
        int direction = getDir(velMsg->linear.x); // 1 forwars -1 back
		//TODO: handle direction
        output = steer(output, direction * velMagnitude, turnAngle);
    } else {
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
