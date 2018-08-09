/*
 * Converts CMD_VEL vectors into joint positions
 * For now it is only used for the simulator, but could become main algorithm for steering bluetounge 2.0
 * Original Author: Sajid Ibne Anower
 * Editors:
 * Date Started: 8/02/2015
 * ros_package: owr_drive_controls
 * ros_node: cmd_vel_2_joints
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define TOPIC "/cmd_vel"

class fourWheelDrive {
    
    public:
        fourWheelDrive();
        void run();
        
    protected:
        void reciveVelMsg(const geometry_msgs::Twist::ConstPtr& velMsg);

    private:
        ros::NodeHandle nh;
        ros::Subscriber cmdVelSub;
        
        ros::Publisher frontLeftDrive;
        ros::Publisher frontRightDrive;
        ros::Publisher backLeftDrive;
        ros::Publisher backRightDrive;
        ros::Publisher frontLeftSwerve;
        ros::Publisher frontRightSwerve;
        ros::Publisher backLeftSwerve;
        ros::Publisher backRightSwerve;
        
        double frontLeftMotorV, frontRightMotorV, backLeftMotorV, backRightMotorV;
        double frontLeftAng, frontRightAng;
        double backLeftAng, backRightAng;
};
