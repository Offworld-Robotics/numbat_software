/*
 * Date Started: 11/11/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Class that handles merging a camera feed and a point cloud input
 */

#include "owr_3d_fusion/CameraFusionNode.h"


int main(int argc, char ** argv) {
    
}


CameraFusionNode::CameraFusionNode() {

}

CameraFusionNode::~CameraFusionNode() {

}

sensor_msgs::PointCloud CameraFusionNode::doColouring ( pcl::PointCloud< pcl::PointXYZRGB > pc, sensor_msgs::Image img ) {
    return sensor_msgs::PointCloud();
}

pcl::PointCloud< pcl::PointXYZRGB > CameraFusionNode::getLatestPointCloud() {
    return pcl::PointCloud<pcl::PointXYZRGB>();
}

void CameraFusionNode::imageCallback ( const sensor_msgs::Image::ConstPtr& frame ) {

}

void CameraFusionNode::pointCloudCallback ( const sensor_msgs::PointCloud::ConstPtr& pc ) {

}

