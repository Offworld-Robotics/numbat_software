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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>



class CameraFusionNode {
    public:
        CameraFusionNode();
        ~CameraFusionNode();
        
        void imageCallback(const sensor_msgs::Image::ConstPtr& frame);
        void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& pc);
        
    private:
        pcl::PointCloud<pcl::PointXYZRGB> getLatestPointCloud();
        sensor_msgs::PointCloud doColouring(pcl::PointCloud<pcl::PointXYZRGB> pc, sensor_msgs::Image img);
        
        ros::NodeHandle nh;
        ros::Publisher colourPub;
        
//         tf::MessageFilter<sensor_msgs::Image> frameCamFilter;
        tf::TransformListener camTFListener;
        ros::Subscriber pcSub;
        ros::Subscriber camSub;
        
        
};

#endif // CAMERA_FUSION_NODE_H
