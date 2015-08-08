/*
	Class header for FineControlNode
	The node manages input into the FineControl GUI from ROS
	By Harry J.E Day for Bluesat OWR <Harry@dayfamilyweb.com>
*/
 
#ifndef FINECONTROLNODE_H
#define FINECONTROLNODE_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include "FineControlGUI.h"

// The message structs needed for availableFeeds
#include "owr_messages/activeCameras.h"
#include "owr_messages/stream.h"

class FineControlNode {

	public:
		FineControlNode(FineControlGUI *gui);
		void spin();
		void receiveFeedsStatus(const owr_messages::activeCameras::ConstPtr &msg);
		void receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void receiveVideoMsg0(const sensor_msgs::Image::ConstPtr& msg);
		void receiveVideoMsg1(const sensor_msgs::Image::ConstPtr& msg);
		
	private:
		FineControlGUI *gui;
		ros::Subscriber gpsSub;
		ros::Subscriber videoSub[TOTAL_FEEDS];
		ros::Subscriber feedsSub;
		
		float voltage;
		ArmState armState;
		float pH;
		float humidity;
		vector3D currentPos;
		float heading;
		float tiltX;
		float tiltY;
		float ultrasonic;
		unsigned char feeds[TOTAL_FEEDS];
};

#endif
