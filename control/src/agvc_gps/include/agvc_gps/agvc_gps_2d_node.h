
#ifndef AGVC_GPS_2D_NODE_H_
#define AGVC_GPS_2D_NODE_H_

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h> 

class AGVCGPS2DNode
{

public:

  AGVCGPS2DNode(); 

  void spin();

private:

  void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr & nav_sat_fix);

  ros::NodeHandle node_handle_;

  ros::Publisher gps_2d_pub_;

  ros::Subscriber gps_sub_;

};

#endif

