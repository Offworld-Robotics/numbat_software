/*
 * GPSLogger Node
 * Logs GPS input to KML
 * By Harry J.E Day for Bluesat OWR
 * Date: 31/05/2014
 */
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <cstring>
#include <cstdio>
#include "AnalysisNode.h"

AnalysisNode::AnalysisNode(AnalysisGUI *newgui) {
	ROS_INFO("Starting Analysis Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
	
	ultrasonic = 0;
	pH = 0;
	humidity = 0;
	
	//pass the function that is called when a message is received
	gpsSub = n.subscribe("/gps/fix", 1000, &AnalysisNode::receiveGpsMsg, this);
	//siteSub = n.subscribe("/gps/fix", 1000, &AnalysisNode::reciveSiteMsg, this);
	videoSub = n.subscribe("/camera/image_raw", 1000, &AnalysisNode::receiveVideoMsg, this);
}

void AnalysisNode::spin() {
	ros::spin();
}

void AnalysisNode::receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received GPS message");
	//ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
	
	latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
	
	gui->updateSiteConstants(latitude, longitude, altitude, pH, ultrasonic, humidity, NULL);
}

/*void GPSGUI::receiveSiteMsg(const bluesat_owr_protobuf::& msg) {
	assert(msg);
	
	ROS_INFO("received a message");
	ROS_INFO("pH %f, ultrasonic %f", msg->pH, msg->ultrasonic);
	
	pH = msg->pH;
	ultrasonic = pH->ultrasonic;
	
	updateSiteConstants(latitude, longitude, altitude, pH, ultrasonic, humidity, NULL);
	
}*/

void AnalysisNode::receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateSiteConstants(latitude, longitude, altitude, pH, ultrasonic, humidity, (unsigned char *)msg->data.data());
}

