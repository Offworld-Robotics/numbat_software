/*
	Auto Node
	Handles updates to the Auto GUI
	By Harry J.E Day for Bluesat OWR
	Date: 31/05/2014
*/

#include "AutoNode.h"
#include <fstream>

AutoNode::AutoNode(AutoGUI *newgui) {
	ROS_INFO("Starting Auto Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
	
	// 
	// Subscribe to all relevant topics for information used by the gui
	// pass the function that is called when a message is received into the subscribe function
	// 
	
	gpsSub = n.subscribe("/gps/fix", 1000, &AutoNode::receiveGpsMsg, this); // GPS related data
}

// Spin to wait until a message is received
void AutoNode::spin() {
	ros::spin();
}

void AutoNode::receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
	
	//create a new node
	ListNode l = (ListNode)malloc(sizeof(vector3D));
	l->lat = msg->latitude;
	l->lon = msg->longitude;
	l->alt = msg->altitude;
	gui->updateInfo(l);
}
