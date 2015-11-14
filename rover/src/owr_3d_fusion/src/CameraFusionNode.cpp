/*
 * Date Started: 11/11/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Class that handles merging a camera feed and a point cloud input
 */

#include "owr_3d_fusion/CameraFusionNode.h"
#include "owr_3d_fusion/logitechC920.h"


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
    pcl::PointCloud<pcl::PointXYZRGB> pc = getLatestPointCloud();
    int x,y;
    float deltaX, deltaY;
    const float focalLengthPx = (PIXEL_TO_M_RATIO/FOCAL_LENGTH_M);
    for(x = 0; x < frame->width; x++) {
        deltaX = tanh(x*focalLengthPx);
        for(y = 0; y < frame->height; y++) {
            deltaY = tanh(y*focalLengthPx);
            //gradient of z is the minimum resolution on the z axis of the point cloud
            float px, py, pz;
            float dist = 0;
#define RES 1
            //TODO: 
            for(dist = 0;dist < 100; dist+=RES) {
                px = deltaX * RES;
                py = deltaY * RES;
                pz = dist;
                pc.at(px,py,px);
            }
        }
    }
}

void CameraFusionNode::pointCloudCallback ( const sensor_msgs::PointCloud::ConstPtr& pc ) {
    
}

