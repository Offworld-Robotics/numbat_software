/*
	Arm Widget Node
	Handles updates to the Arm Widget GUI
	By Harry J.E Day for Bluesat OWR
	Date: 31/05/2014
*/

#include "ArmWidgetNode.h"
#include <fstream>

ArmWidgetNode::ArmWidgetNode(ArmWidgetGUI *newgui) {
	ROS_INFO("Starting Arm Widget Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
	
	// 
	// Subscribe to all relevant topics for information used by the gui
	// pass the function that is called when a message is received into the subscribe function
	// 
	
	tfSub = n.subscribe("/gps/fix", 1000, &ArmWidgetNode::receiveTFMsg, this); // TFs
}

// Spin to wait until a message is received
void ArmWidgetNode::spin() {
	ros::spin();
}

void ArmWidgetNode::receiveTFMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
//	gui->
}
