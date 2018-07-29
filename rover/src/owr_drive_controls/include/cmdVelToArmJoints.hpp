/*
 * Converts CMD_VEL vectors into arm joint positions
 * 
 * Original Author: Elliott Smiht
 * Editors:
 * Date Started: 2/6/18
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define TOPIC "/cmd_arm_vel"

class CmdVelToArmJoints {
    public:
	CmdVelToArmJoints();
	void run();
    protected:
        void receiveArmVelMsg(const geometry_msgs::Twist::ConstPtr& stick);
        
    private:
        ros::NodeHandle nh;
        ros::Subscriber cmdArmVelSub;
        
        ros::Publisher armUpper;
        ros::Publisher armLower;
	ros::Publisher armBaseRotatePub;
	ros::Publisher clawGripPub;
	ros::Publisher clawTwistPub;
        
        double armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist;
};

  