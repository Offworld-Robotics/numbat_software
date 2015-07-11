#ifndef BUTTERCUP_NODE_H
#define BUTTERCUP_NODE_H

#include <stdint.h>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#define BAUD B115200
#define TICKS_PER_REV 360.0

#define SAFETY_OFF 0
#define SAFETY_ON 1
#define SAFETY_FLASHING 2

class ButtercupNode
{
	public:
		ButtercupNode();

	private:
		void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void safetyCallback(const std_msgs::Int32::ConstPtr& msg);

		void spin(); 
		void publishOdom(int32_t lDisp, int32_t rDisp, int16_t lVel, int16_t rVel);
		void publishBattery(uint16_t battery, uint16_t reference);
		void publishEstop(uint16_t estop);
		void publishButter(std::string bollocks);
		int openSerial(std::string port);
		void buttercupComm(bool safety);

		ros::NodeHandle nodeHandle;
		ros::Publisher odomPub;
		ros::Publisher odomRawPub;
		ros::Publisher batteryPub;
		ros::Publisher estopPub;
		ros::Publisher butterPub;
		ros::Subscriber cmdVelSub;
		ros::Subscriber safetySub;
		
		geometry_msgs::Twist cmdVelBuf;
		int safety;
		std_msgs::Float64MultiArray odomRaw;
		int estop;

		int ser;

		double width;
		double wheelCircumference;

		uint16_t P, I, D;

};

#endif
