/*
 * Date Started: 11/11/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Class that handles merging a camera feed and a point cloud input
 */
#ifndef CAMERA_FUSION_NODE_H
#define CAMERA_FUSION_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "owr_3d_fusion/Octree.h"
#include "owr_3d_fusion/CPURayTracer.hpp"

#define TOPIC "/pcl/colourFuse0"
#define PCL_IN_TOPIC "/pcl"




class CameraFusionNode {
    public:
        CameraFusionNode();
        ~CameraFusionNode();
        
        void imageCallback(const sensor_msgs::Image::ConstPtr& frame);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
        
        void spin();
        
    private:
        pcl::PointCloud<pcl::PointXYZRGB> getLatestPointCloud();
        sensor_msgs::PointCloud2 doColouring(pcl::PointCloud<pcl::PointXYZRGB> pc, sensor_msgs::Image img);
        
        ros::NodeHandle nh;
        ros::Publisher colourPub;
        
//         tf::MessageFilter<sensor_msgs::Image> frameCamFilter;
        tf::TransformListener camTFListener;
        ros::Subscriber pcSub;
        ros::Subscriber camSub;
        
        CPURayTracer tracer;
        Octree tr;
        
        
};

#endif // CAMERA_FUSION_NODE_H
