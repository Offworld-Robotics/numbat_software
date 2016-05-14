/*
	Class header for AutoMapNode
	The node manages input into the AutoMap GUI
	By Yiwei
*/
 
#ifndef AutoMapNODE_H
#define AutoMapNODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "AutoMapGUI.h"

class AutoMapNode {
	public:
		AutoMapNode(AutoMapGUI *gui);
		void spin();
		void receiveOccGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		
	private:
		AutoMapGUI *gui;
		ros::Subscriber occGridSub;
};

#endif
