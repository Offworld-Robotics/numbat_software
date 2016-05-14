/*
	AutoMap Node
	Handles updates to the AutoMap GUI
	By Yiwei Han
*/

#include "AutoMapNode.h"
#include <fstream>

AutoMapNode::AutoMapNode(AutoMapGUI *newgui) {
	ROS_INFO("Starting AutoMap Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
    	
	occGridSub = n.subscribe("/map", 1000, &AutoMapNode::receiveOccGridMsg, this);
}

// Spin to wait until a message is received
void AutoMapNode::spin() {
	ros::spin();
}

void AutoMapNode::receiveOccGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	assert(msg);
	
	ROS_INFO("received grid message");
	ROS_INFO("grid width: %d, height: %d", msg->info.width, msg->info.height);
	gui->updateGrid((char *)msg->data.data(), msg->info.width, msg->info.height);
}
