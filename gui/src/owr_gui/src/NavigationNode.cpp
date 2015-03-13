/*
	Navigation Node
	Handles updates to the Navigation GUI
	By Harry J.E Day for Bluesat OWR
	Date: 31/05/2014
*/

#include "NavigationNode.h"
#include <fstream>

NavigationNode::NavigationNode(NavigationGUI *newgui) {
	ROS_INFO("Starting Navigation Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
	
	battery = 5;
	signal = 5;
	tiltX = 30;
	tiltY = 30;
	ultrasonic = 0;
	altitude = 0;
	
	// pass the function that is called when a message is received
	gpsSub = n.subscribe("/gps/fix", 1000, &NavigationNode::receiveGpsMsg, this); // GPS related data
	batterySub = n.subscribe("/status/battery", 1000, &NavigationNode::receiveBatteryMsg, this); // Power left on the battery
	videoSub = n.subscribe("/camera/image_raw", 1000, &NavigationNode::receiveVideoMsg, this); // Frames of video from camera
}

void NavigationNode::spin() {
	ros::spin();
}

void NavigationNode::receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
		
	//create a new node
	ListNode l = (ListNode)malloc(sizeof(vector2D));
	l->y = msg->latitude;
	l->x = msg->longitude;
	altitude = msg->altitude;
	gui->updateInfo(battery, signal, ultrasonic, l, altitude, target);
}


void NavigationNode::receiveBatteryMsg(const bluesat_owr_protobuf::battery_ros::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("voltage %f", msg->voltage);
	battery = msg->voltage;
	
	gui->updateInfo(battery, signal, ultrasonic, NULL, altitude, target);
}

void NavigationNode::receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height);
}
