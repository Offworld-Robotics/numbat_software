/*
 * This manages input into the GUI from ROS
 * By Harry J.E Day for BlueSat OWR <Harry@dayfamilyweb.com>
 */

#ifndef ANALYSISGUI_H
#define ANALYSISGUI_H

#include "site.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include "SiteGui.h"

class ANALYSISGUI {

	public:
		ANALYSISGUI(SiteGui *gui);
		void spin();
		void reciveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		//void reciveStatsMsg();// pH, ultrasonic, humidity
		void reciveVideoMsg(const sensor_msgs::Image::ConstPtr& msg);
		
	private:
		SiteGui *gui;
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
