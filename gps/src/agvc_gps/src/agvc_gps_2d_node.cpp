
#include <agvc_gps/agvc_gps_2d_node.h>

AGVCGPS2DNode::AGVCGPS2DNode()
{

  ros::NodeHandle private_node_handle("~");

  // Read in GPS topic to subscribe to.
  std::string gps_topic("fix");
  if (!private_node_handle.getParam("gps_fix_topic", gps_topic))
  {
    ROS_WARN("[AGVC GPS 2D Node] : <gps_fix_topic> not specified, defaulting to %s", gps_topic.c_str());
  }
  else
  {
    ROS_INFO("[AGVC GPS 2D Node] : <gps_fix_topic> set to %s", gps_topic.c_str());
  }

  // Subscribe to the topic.
  gps_sub_ = node_handle_.subscribe(gps_topic, 20, &AGVCGPS2DNode::navSatFixCallback, this);

  // Read in GPS topic to publish.
  std::string gps_2d_topic("fix_2d");
  if (!private_node_handle.getParam("gps_fix_2d_topic", gps_2d_topic))
  {
    ROS_WARN("[AGVC GPS 2D Node] : <gps_fix_2d_topic> not specified, defaulting to %s", gps_2d_topic.c_str());
  }
  else
  {
    ROS_INFO("[AGVC GPS 2D Node] : <gps_fix_2d_topic> set to %s", gps_2d_topic.c_str());
  }

  // Advertise topic.
  gps_2d_pub_ = node_handle_.advertise<sensor_msgs::NavSatFix>(gps_2d_topic, 20);

}

void AGVCGPS2DNode::spin()
{
  
  ros::spin();

}

void AGVCGPS2DNode::navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr & nav_sat_fix)
{

  sensor_msgs::NavSatFix nav_sat_fix_2d;

  nav_sat_fix_2d.header = nav_sat_fix->header;
  nav_sat_fix_2d.latitude = nav_sat_fix->latitude;
  nav_sat_fix_2d.longitude= nav_sat_fix->longitude;

  // Set altitude to zero.
  nav_sat_fix_2d.altitude= 0.0f;

  nav_sat_fix_2d.position_covariance = nav_sat_fix->position_covariance;

  // Large altitude uncertainty.
  nav_sat_fix_2d.position_covariance[8] = 1e9;

  nav_sat_fix_2d.position_covariance_type = nav_sat_fix->position_covariance_type;

  gps_2d_pub_.publish(nav_sat_fix_2d);

}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "agvc_gps_2d_node");

  AGVCGPS2DNode agvc_gps_2d_node;

  agvc_gps_2d_node.spin();

  return 0;
  
}

