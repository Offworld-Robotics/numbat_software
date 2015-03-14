#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "bluesat_owr_protobuf/battery_ros.h"

typedef struct _gpspoint {
	double longitude;
	double latitude;
} gpspoint;


#define POINTS 16
static gpspoint gpslist[] = {
{-33.918149, 151.232053},
{-33.918154, 151.231992},
{-33.918152, 151.231936},
{-33.918156, 151.231867},
{-33.918152, 151.231803},
{-33.918151, 151.231740},
{-33.918144, 151.231693},
{-33.918145, 151.231652},
{-33.918148, 151.231592},
{-33.918144, 151.231531},
{-33.918141, 151.231449},
{-33.918139, 151.231386},
{-33.918130, 151.231305},
{-33.918130, 151.231225},
{-33.918129, 151.231139},
{-33.918129, 151.231053},
};

int main(int argc, char ** argv) {
	 ros::init (argc, argv, "dataGen");
	
	ros::NodeHandle node;
	
	//publisher
	ros::Publisher pub = node.advertise<sensor_msgs::NavSatFix>("/gps/fix",  1000);
	ros::Publisher pub2 = node.advertise<bluesat_owr_protobuf::battery_ros>("/status/battery",  1000);
	 
	ros::Rate loop_rate(1);
	
	int i = 0;
	bool dir = true; // true->inc i
	float voltage = 10;
	while(ros::ok()) { //loop whilst ros is running
		//indicate we are sending
		ROS_INFO("Sending\n"); 
		
		sensor_msgs::NavSatFix msg;
		msg.longitude = gpslist[i].longitude;
		msg.latitude = gpslist[i].latitude;
		msg.altitude = 0;
		if (dir) {
			i++;
		} else {
			i--;
		}
		if (i == POINTS) {
			i--;
			dir = false;
		} else if (i < 0) {
			i++;
			dir = true;
		}
				
		pub.publish(msg);
		
		bluesat_owr_protobuf::battery_ros msg2;
		msg2.voltage = voltage;
		pub2.publish(msg2);
		
		loop_rate.sleep();
	}
}
