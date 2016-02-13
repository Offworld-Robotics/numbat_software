/*
	Class header for ArmWidgetNode
	The node manages input into the Arm Widget GUI from ROS
	By Harry J.E Day for Bluesat OWR <Harry@dayfamilyweb.com>
*/
 
#ifndef ARMWIDGETNODE_H
#define ARMWIDGETNODE_H

#include "ListNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "ArmWidgetGUI.h"

class ArmWidgetNode {

	public:
		ArmWidgetNode(ArmWidgetGUI *gui);
		void spin();
		void receiveTFMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		
	private:
		ArmWidgetGUI *gui;
		ros::Subscriber tfSub;
};

#endif
