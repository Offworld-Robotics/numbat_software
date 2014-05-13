
#ifndef AGVC_GPS_ORIGIN_NODE_H_
#define AGVC_GPS_ORIGIN_NODE_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <agvc_gps/ConvertGPSOrigin.h>

class AGVCGPSOriginNode
{

public:

  AGVCGPSOriginNode();

  void spin();

private:

  void waitForValidGPS();

  void gpsCallback(const nav_msgs::Odometry::ConstPtr & gps);
  bool gpsServiceCallback(agvc_gps::ConvertGPSOrigin::Request & req, agvc_gps::ConvertGPSOrigin::Response & res);

  void convertGPSOrigin(const nav_msgs::Odometry & gps_global, nav_msgs::Odometry & gps_local);

  ros::NodeHandle node_handle_;

  ros::Publisher gps_local_pub_;
  ros::Subscriber gps_global_sub_;

  ros::ServiceServer gps_origin_server_;

  double origin_x_;
  double origin_y_;

  bool have_origin_;

};

#endif

