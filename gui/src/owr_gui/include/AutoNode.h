/*
	Class header for AutoNode
	The node manages input into the Auto GUI from ROS
	By Harry J.E Day for Bluesat OWR <Harry@dayfamilyweb.com>
*/
 
#ifndef AutoNODE_H
#define AutoNODE_H

#include "ListNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "AutoGUI.h"

class AutoNode {

	public:
		AutoNode(AutoGUI *gui);
		void spin();
		void receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		
	private:
		AutoGUI *gui;
		ros::Subscriber gpsSub;
		
		// ultrasonic vector here...
};

#endif
