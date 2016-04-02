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
#include "NavigationGUI.h"
#include <image_transport/image_transport.h>

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
		
	private:
		NavigationGUI *gui;
		ros::Subscriber gpsSub;
		ros::Subscriber batterySub;
		image_transport::Subscriber videoSub[TOTAL_FEEDS];
		ros::Subscriber feedsSub;
		
		float battery;
		float signal;
		float tiltX;
		float tiltY;
		float ultrasonic;
		double altitude;
		unsigned char feeds[TOTAL_FEEDS];
		vector3D target;
};

#endif
