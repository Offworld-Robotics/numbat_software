/*
 * Converts CMD_VEL vectors into joint positions
 * For now it is only used for the simulator, but could become main algorithm for steering bluetounge 2.0
 * Original Author: Harry J.E Day
 * Editors:
 * Date Started: 8/02/2015
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 */

/*Constants that are needed for this (eventually we should probably calculate these from transforms):

    ROVER_CENTER_2_WHEEL_Y := 400.23mm == 0.40023 (distance from base_link_origin to back_right_wheel shaft)
    BACK_WHEEL_SPAN :=802.73mm == 0.80273
    HALF_ROVER_WIDTH_X := 271.30mm == .27130 (distance from base_link_origin to back_right_wheel shaft)
    FRONT_W_2_BACK_W_X := 542.16mm == 0.54216
*/
#define ROVER_CENTER_2_WHEEL_Y 0.40023
#define BACK_WHEEL_SPAN 0.80273
#define HALF_ROVER_WIDTH_X .27130
#define FRONT_W_2_BACK_W_X 0.54216

#include "cmdVelToJoints.hpp"
#include "math.h"

#include <geometry_msgs/Vector3.h>

CmdVelToJoints::CmdVelToJoints() {

}

//for a full explanation of this logic please see the scaned notes at
//https://bluesat.atlassian.net/browse/OWRS-203
void CmdVelToJoints::reciveVelMsg ( const geometry_msgs::Twist::ConstPtr& velMsg ) {
    const double turnAngle = atan(velMsg.linear.x/velMsg.linear.y);
    const double rotationRadius = HALF_ROVER_WIDTH_X/sin(turnAngle);
    geometry_msgs::Vector3 rotationCentre;
    rotationCentre.x = -HALF_ROVER_WIDTH_X;
    rotationCentre.y = sqrt(pow(rotationRadius,2)+pow(HALF_ROVER_WIDTH_X,2));
    const double angularVelocity = velMsg.linear.x / rotationRadius;
    
    
}

