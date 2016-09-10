/*
	Class header for NavigationNode
	The node manages input into the Navigation GUI from ROS
	By Harry J.E Day for Bluesat OWR <Harry@dayfamilyweb.com>
*/
 
#ifndef NAVIGATIONNODE_H
#define NAVIGATIONNODE_H

#include "ListNode.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include "owr_messages/status.h"
#include "owr_messages/voltage.h"
#include "owr_messages/adc.h"
#include "NavigationGUI.h"
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <string>
#include <algorithm> // for finding the right value	
// The message structs needed for availableFeeds
#include "owr_messages/activeCameras.h"
#include "owr_messages/stream.h"


class NavigationNode {

	public:
		NavigationNode(NavigationGUI *gui);
		void spin();
		void receiveFeedsStatus(const owr_messages::activeCameras::ConstPtr &msg);
		void receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void receiveBatteryMsg(const owr_messages::status::ConstPtr& msg);
		void receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg);
		void receiveVoltageMsg(const std_msgs::Float32::ConstPtr& msg);
		void receiveClawMsg(const owr_messages::adc::ConstPtr& msg);
		void receiveLidarMsg(const std_msgs::Int16::ConstPtr& msg);
		
	private:
		NavigationGUI *gui;
		ros::Subscriber gpsSub;
		ros::Subscriber batterySub;
		
		ros::Subscriber lidarModeSub;
		
		image_transport::Subscriber videoSub[TOTAL_FEEDS];
		ros::Subscriber feedsSub;
		ros::Subscriber voltSub; // voltmeter callback
		ros::Subscriber adcSub; // adc callback

		float battery;
		float signal;
		float tiltX;
		float tiltY;
		float ultrasonic;
		float lidar;
		float voltage;
		std::vector<float> pot_vals;
		std::vector<std::string> pot_names;
		double altitude;
		unsigned char feeds[TOTAL_FEEDS];
		vector3D target;
};

#endif
