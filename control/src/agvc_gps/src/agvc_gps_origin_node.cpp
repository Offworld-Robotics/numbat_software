
#include <agvc_gps/agvc_gps_origin_node.h>

AGVCGPSOriginNode::AGVCGPSOriginNode() : have_origin_(false)
{

  ros::NodeHandle private_node_handle("~");

  // Read in GPS topic to subscribe to.
  std::string gps_global_topic("fix");
  if (!private_node_handle.getParam("gps_global_topic", gps_global_topic))
  {
    ROS_WARN("[AGVC GPS Origin Node] : <gps_global_topic> not specified, defaulting to %s", gps_global_topic.c_str());
  }
  else
  {
    ROS_INFO("[AGVC GPS Origin Node] : <gps_global_topic> set to %s", gps_global_topic.c_str());
  }

  // Subscribe to the topic.
  gps_global_sub_ = node_handle_.subscribe(gps_global_topic, 20, &AGVCGPSOriginNode::gpsCallback, this);

  // Read in GPS topic to publish.
  std::string gps_local_topic("fix_origin");
  if (!private_node_handle.getParam("gps_local_topic", gps_local_topic))
  {
    ROS_WARN("[AGVC GPS Origin Node] : <gps_local_topic> not specified, defaulting to %s", gps_local_topic.c_str());
  }
  else
  {
    ROS_INFO("[AGVC GPS Origin Node] : <gps_local_topic> set to %s", gps_local_topic.c_str());
  }

  // Advertise the topic.
  gps_local_pub_ = node_handle_.advertise<nav_msgs::Odometry>(gps_local_topic, 20);

   // Read in GPS origin service name.
  std::string gps_origin_service_name("fix");
  if (!private_node_handle.getParam("gps_origin_service_name", gps_origin_service_name))
  {
    ROS_WARN("[AGVC GPS Origin Node] : <gps_origin_service_name> not specified, defaulting to %s", gps_origin_service_name.c_str());
  }
  else
  {
    ROS_INFO("[AGVC GPS Origin Node] : <gps_origin_service_name> set to %s", gps_origin_service_name.c_str());
  }

  // Advertise service to convert GPS coordinates.
  gps_origin_server_ = node_handle_.advertiseService(gps_origin_service_name, &AGVCGPSOriginNode::gpsServiceCallback, this);

}

void AGVCGPSOriginNode::spin()
{

  waitForValidGPS();

  ros::spin();

}

void AGVCGPSOriginNode::waitForValidGPS()
{

  ros::Rate r(1);

  while (ros::ok() && !have_origin_)
  {
    ROS_INFO("[AGVC GPS Origin Node] : waiting for valid GPS");
    ros::spinOnce();
    r.sleep();
  }

}

void AGVCGPSOriginNode::gpsCallback(const nav_msgs::Odometry::ConstPtr & gps)
{

  // We haven't got a valid fix yet.
  if (!have_origin_)
  {
    // Fix valid.
    if (gps->pose.covariance[0] > 0.0f)
    {
      origin_x_ = gps->pose.pose.position.x;
      origin_y_ = gps->pose.pose.position.y;
      have_origin_ = true;
      ROS_INFO("[AGVC GPS Origin Node] : GPS origin set to (%f, %f)", origin_x_, origin_y_);
    }
    // Fix not valid.
    else
    {
      return;
    }
  }

  // Convert GPS reading to local coordinates and republish.
  nav_msgs::Odometry gps_local;
  convertGPSOrigin(*gps, gps_local);
  gps_local_pub_.publish(gps_local);

}

bool AGVCGPSOriginNode::gpsServiceCallback(agvc_gps::ConvertGPSOrigin::Request & req, agvc_gps::ConvertGPSOrigin::Response & res)
{

  if (have_origin_)
  {
    convertGPSOrigin(req.gps_global, res.gps_local);
    return true;
  }
  else
  {
    return false;
  }

}

void AGVCGPSOriginNode::convertGPSOrigin(const nav_msgs::Odometry & gps_global, nav_msgs::Odometry & gps_local)
{

  gps_local.header = gps_global.header;
  gps_local.child_frame_id = gps_global.child_frame_id;
  gps_local.pose.pose = gps_global.pose.pose;

  // Convert to local coordinates.
  gps_local.pose.pose.position.x -= origin_x_;
  gps_local.pose.pose.position.y -= origin_y_;

  gps_local.pose.covariance = gps_global.pose.covariance;
  gps_local.twist.twist = gps_global.twist.twist;
  gps_local.twist.covariance = gps_global.twist.covariance;

}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "agvC_gps_origin_node");

  AGVCGPSOriginNode agvc_gps_origin_node;

  agvc_gps_origin_node.spin();

  return 0;

}
