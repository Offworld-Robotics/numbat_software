/*
	Class header for NavigationNode
	The node manages input into the Navigation GUI from ROS
	By Harry J.E Day for Bluesat OWR <Harry@dayfamilyweb.com>
*/
 
#ifndef NAVIGATIONNODE_H
#define NAVIGATIONNODE_H

#include "ListNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include "bluesat_owr_protobuf/battery_ros.h"
#include "NavigationGUI.h"

class NavigationNode {

	public:
		NavigationNode(NavigationGUI *gui);
		void spin();
		void receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void receiveBatteryMsg(const bluesat_owr_protobuf::battery_ros::ConstPtr& msg);
		void receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg);
		
	private:
		NavigationGUI *gui;
		ros::Subscriber gpsSub;
		ros::Subscriber batterySub;
		ros::Subscriber videoSub;
		
		float battery;
		float signal;
		float tiltX;
		float tiltY;
		float ultrasonic;
		double altitude;
		vector2D target;
};

#endif
