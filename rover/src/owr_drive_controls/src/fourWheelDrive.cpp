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
 * Desciption: translate the current vels with the new turn_angle
 */

motorVels FourWheelDrive::steer(motorVels vels, double velMagnitude, double turn_angle) {

    // Capping the turnangle to 45% of M_PI/2
    if(turn_angle >= FWDRIVE_LIMIT) {
        turn_angle = FWDRIVE_LIMIT;
    } else if(turn_angle <= -FWDRIVE_LIMIT) {
        turn_angle = -FWDRIVE_LIMIT;
    }

	// Primary Circle
		// radius: (cos(90 - turn_angle) * ROVER_HEIGHT)/2
		// Assume origin at the center of the rover
		// center: at ((ROVER_HEIGHT*tan(90-turn_angle))/2, 0)

	// Secondary Circle
		// using pythag
		// radius^2 = (ROVER_WIDTH+center)^2 + (ROVER_HEIGHT/2)^2
		// center: same as primary

	// Turn angles
		// wheels on the primary circle
			// front wheel: turn_angle
			// back wheel: -turn_angle
		// wheels on the secondary cicle
			// tan(90 - fi) = (ROVER_WIDTH + center)/(ROVER_HEIGHT/2)
			// front wheel: fi
			// back wheel: -fi

    motorVels output = vels; 
    double center = (ROVER_HEIGHT/2) * tan(90-turn_angle);
    double primary_radius = (ROVER_HEIGHT/2) / cos(DEG90 - turn_angle);
    double secondary_radius = sqrt( pow(ROVER_WIDTH+center, 2) + pow(ROVER_HEIGHT/2, 2));
    double w = velMagnitude/primary_radius;

    ROS_INFO("center of rotation at %lf", center);
    ROS_INFO("primary radius: %lf",primary_radius);
    ROS_INFO("secondary radius %lf",secondary_radius);
    if(0 < turn_angle) {
	    ROS_INFO("Right turn at %lf", turn_angle);
	    // Turn Angles
	    output.frontRightAng = turn_angle;
	    output.backRightAng = -turn_angle;

	    output.frontLeftAng = DEG90 - atan2(ROVER_WIDTH + center, ROVER_HEIGHT/2);
	    output.backLeftAng = -output.frontLeftAng;

	    output.backRightMotorV = velMagnitude;
	    output.frontRightMotorV = velMagnitude;
	    // v = w1*r
	    output.frontLeftMotorV = w*secondary_radius;
	    output.backLeftMotorV = w*secondary_radius;

    } else if(turn_angle > M_PI) {
	    ROS_INFO("Left turn at %lf", turn_angle);
	    output.frontLeftAng = turn_angle;
	    output.backLeftAng = -turn_angle;

	    output.frontRightAng = DEG90 - atan2(ROVER_WIDTH + center, ROVER_HEIGHT/2);
	    output.backRightAng = -output.frontRightAng;

	    output.backLeftMotorV = velMagnitude;
	    output.frontLeftMotorV = velMagnitude;
	    // w1 = v/r		
	    // v = w1*r
	    output.frontRightMotorV = w*secondary_radius;
	    output.backRightMotorV = w*secondary_radius;

    } else {
	    // doVelTranslation takes care of straight pathing 
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
        const double turn_angle = atan2(velMsg->linear.y, fabs(velMsg->linear.x));
        int direction = getDir(velMsg->linear.x); // 1 forwars -1 back
		//TODO: handle direction
        output = steer(output, direction * velMagnitude, turn_angle);
    } else {
        ROS_INFO("drive straight");
        output.frontLeftMotorV = output.backLeftMotorV = velMsg->linear.x;
        output.frontRightMotorV = output.backRightMotorV = velMsg->linear.x;
        output.frontRightAng = output.frontLeftAng = 0;
        output.backRightAng = output.backLeftAng = 0;
    }
    return output;
}
