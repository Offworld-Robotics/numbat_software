/*
 * Converts CMD_VEL vectors into joint positions
 * For now it is only used for the simulator, but could become main algorithm for steering bluetounge 2.0
 * Original Author: Harry J.E Day
 * Editors:
 * Date Started: 8/02/2015
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define TOPIC "/cmd_vel"

class CmdVelToJoints {
    
    public:
        CmdVelToJoints();
        
    protected:
        void reciveVelMsg(const geometry_msgs::Twist::ConstPtr& velMsg);
        
    private:
        ros::NodeHandle nh;
        ros::Subscriber cmdVelSub;
        
    
};