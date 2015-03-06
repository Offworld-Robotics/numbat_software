/*
 * This manages input into the GUI from ROS
 * By Harry J.E Day for BlueSat OWR <Harry@dayfamilyweb.com>
 */

#ifndef ANALYSISNODE_H
#define ANALYSISNODE_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include "AnalysisGUI.h"

class AnalysisNode {

	public:
		AnalysisNode(AnalysisGUI *gui);
		void spin();
		void receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		//void receiveStatsMsg();// pH, ultrasonic, humidity
		void receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg);
		
	private:
		AnalysisGUI *gui;
		ros::Subscriber gpsSub;
		//ros::Subscriber statsSub;
		ros::Subscriber videoSub;
		double latitude;
		double longitude;
		float altitude;
		float pH;
		float ultrasonic;
		float humidity;
};

#endif
