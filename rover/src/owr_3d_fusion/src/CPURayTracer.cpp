/*
 * Date Started: 26/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: A simple CPU based ray tracer used for testing
 */

#include "owr_3d_fusion/CPURayTracer.hpp"
#include </home/ros/VirtualBox-4.3.12/src/VBox/Additions/x11/x11include/mesa-7.2/src/mesa/math/m_norm_tmp.h>
#include <math.h>
#include "owr_3d_fusion/logitechC920.h"

CPURayTracer::CPURayTracer() : RayTracer() {

}

CPURayTracer::~CPURayTracer(): RayTracer() {

}


void CPURayTracer::runTraces() {

    int x, y;
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    //our accuracy does not require double precision
    float deltaX, deltaY;
    //calc this here so it only does the math once, #defines will run this many time
    const float focalLengthPx = (PIXEL_TO_M_RATIO/FOCAL_LENGTH_M);
    //NOTE: this is not the most efficient way to do this
    //but it is the simplest
    //see: http://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html
    //search the image
    for(x = 0; x < image.rows; x++) {
        deltaX = tanh(x*focalLengthPx);
        for(y = 0; y < image.cols; y++) {
            cv::Vec3i pt = image.at<cv::Vec3i>(x,y);
            deltaY = tanh(y*focalLengthPx);
            //gradient of z is the minimum resolution on the z axis of the point cloud
            simplePoint target;
            float dist = 0;

            for(dist = 0;dist < 100; dist+=RES) {
                target.x = deltaX * RES;
                target.y = deltaY * RES;
                target.z = dist;
                octNode node = tree.getNode(target);
            }
        }
    }
}