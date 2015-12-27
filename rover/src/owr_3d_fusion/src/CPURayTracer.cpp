/*
 * Date Started: 26/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: A simple CPU based ray tracer used for testing
 */

#include "owr_3d_fusion/CPURayTracer.hpp"
#include <math.h>
#include "owr_3d_fusion/logitechC920.h"

CPURayTracer::CPURayTracer() : RayTracer() {

}

CPURayTracer::~CPURayTracer() {

}


void CPURayTracer::runTraces() {

    int x, y;
    //our accuracy does not require double precision
    float deltaX, deltaY;
    //calc this here so it only does the math once, #defines will run this many time
    const float focalLengthPx = (PX_TO_M/FOCAL_LENGTH_M);
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
                //if a point is exists at our resolution then we have a match
                if(node.dimensions.x <= RES) {
                    //match
                    //NOTE: we have a loss of accuracy by using the target point here
                    //testing should be done to see if it is better to use the laserScan point.
                    match(target,pt);
                    break;
                //the next size up, match
                } else if (node.dimensions.x <= RES * 8) {
                    if(node.getPointAt(tree.calculateIndex(target, node.orig)).isEmpty()) {
                        match(target,pt);
                        break;
                    }
                } else {
                    //check within required accuracy
                    simplePoint existingPoint = node.getPointAt(tree.calculateIndex(target, node.orig));
                    existingPoint = target-existingPoint;
                    if(fabs(existingPoint.x) <= RES && fabs(existingPoint.y) <= RES && fabs(existingPoint.y) <= RES) {
                        match(target,pt);
                        break;
                    }
                }
            }
        }
    }
}

void CPURayTracer::match ( simplePoint pt, cv::Vec3i pixel ) {
    pcl::PointXYZRGB newPt;
    newPt.x = pt.x;
    newPt.y = pt.y;
    newPt.z = pt.z;
    //BGR
    newPt.b = pixel[0];
    newPt.g = pixel[1];
    newPt.r = pixel[2];
    
    cld.push_back(newPt);
}
