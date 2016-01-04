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

// #define DEBUG

CPURayTracer::CPURayTracer() : RayTracer(), cld(new pcl::PointCloud<pcl::PointXYZRGB> ()) {
    
}

CPURayTracer::~CPURayTracer() {
    cld.reset();
}


void CPURayTracer::runTraces() {
    /*
     * Important:
     * This function uses two co-ordinate systems, the one used by ROS, and the one used by OpenCV
     * OpenCV co-ordinates have the origin in the top-right corner of the image, with x being the horizontal axis
     * and y being the vertical axis. (+X is down, and +Y is right)
     * ROS uses a 3D co-ordinate sytem where the origin is at the center of the image, +X is forward, +Y is left, and +Z is up
     * 
     * So the conversion from openCV to ROS co-ordinates (without projecting) is:
     * 
     *    F(x,y) = (0,-y * pxToM - halfImageHeightInM, -x * pxToM + halfImageHeightInM )
     * 
     * i.e x and y are inverted and offset by half the width of the image. 
     * 
     * Aditionally when we do our projection, our origin in ROS's co-ordinate system is offset by the focal length from the origin on the Z axis.
     * 
     * This function uses floats not double as ROSs co-ordinate system is in M and our maximum resolution is currently 1cm.
     */
    
    cld.reset();
    shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > newCld(new pcl::PointCloud<pcl::PointXYZRGB> ());
    cld = newCld;
    //NOTE: this function unfortuantly has to use two cordinate systems
    //a metric one in meters, and a pixel based one in pixels.
    int pixelX = -1, pixelY = -1;
    float metricY, metricZ;
    float deltaY, deltaZ;
    //calc this here so it only does the math once, #defines will run this many time
    const float focalLengthPx = (PX_TO_M/FOCAL_LENGTH_M);
    #ifdef DEBUG
        std::cout << "focalLengthPx:" << focalLengthPx << "PZ_TO_M" <<PX_TO_M << std::endl;
    #endif
    
    //we redo this function here as it is dependent on the resolution of the image
    const float pxToM = SENSOR_DIAG_M/sqrt(pow(image.cols,2) + pow(image.rows,2)) ;
    //the image co-ordinate system starts in the top right corner, whilst the pcl system starts in the center
    //we need to add an offset
    const float metricYOffset = (image.cols/2.0) * pxToM;
    const float metricZOffset = (image.rows/2.0) * pxToM;
    //NOTE: this is not the most efficient way to do this
    //but it is a simpler way.
    //see: http://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html
    //search the image
    cv::MatIterator_<cv::Vec3b> it, end;
    for(it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>(); it != end; ++it) {
//     for(pixelX = 0; pixelX < image.cols; pixelX++) {
        const cv::Point pos = it.pos();
        pixelX = pos.x;
        if(pixelY != pos.y) {
            pixelY = pos.y;
            metricZ= pixelY * (-pxToM) + metricZOffset;
            deltaZ = tanh(metricZ/focalLengthPx);
        }
        #ifdef DEBUG
            std::cout << "pixel" << pixelX << "," << pixelY << std::endl;
        #endif
        metricY= pixelX * (-pxToM) + metricYOffset;
        deltaY = tanh(metricY/FOCAL_LENGTH_M);
//         for(pixelY = 0; pixelY < image.rows; pixelY++) {
        cv::Vec3b pt = (*it);
       
        #ifdef DEBUG
            std::cout << "pixelY:" << pixelY
                << "deltaZ:" << deltaZ 
                <<  "metricZ:" << metricZ
                << "tanInput:" << metricZ*focalLengthPx << std::endl;
        #endif
        //gradient of z is the minimum resolution on the z axis of the point cloud
        simplePoint target;
        float dist = 0;
        //the node retrived
        octNode node;
//         node.dimensions.x = 0;
//         node.dimensions.y = 0;
//         node.dimensions.z = 0;
        float incDist = RES;
        //this is in metric
        for(dist = FOCAL_LENGTH_M;  //start at the end of the focal length, because of how we calculate everything our z origin is at -f
            dist < TRACE_RANGE;
            //use the larger of the dimension of the cell or half the resolution as an increase
//                 incDist = (RES < (node.dimensions.x/2)) ? (node.dimensions.x/2) : RES, dist+=incDist 
            dist+=RES
        ) {
            target.x = dist - FOCAL_LENGTH_M;
            target.y = deltaY * dist;
            target.z = deltaZ * dist;
            #ifdef DEBUG
                std::cout << target.x << "," << target.y << "," << target.z << " is target" << std::endl;
            #endif
            node = tree->getNode(target);
            #ifdef DEBUG
                std::cout << node.orig.x << "," << node.orig.y << "," << node.orig.z << " is found" << std::endl;
                std::cout << "\t" << node.dimensions.x << "," << node.dimensions.y << "," << node.dimensions.z << " (dims)" << std::endl;
                std::cout << "\tlocCode:" << node.locationCode << " childMask" << node.childrenMask  << std::endl;
            #endif
            
            //if a point is exists at our resolution then we have a match
            if(node.dimensions.x <= RES) {
                //match
                //NOTE: we have a loss of accuracy by using the target point here
                //testing should be done to see if it is better to use the laserScan point.
                #ifdef DEBUG
                    std::cout << "match on small node" << std::endl;
                    std::cout << pixelX << "," << pixelY << std::endl;
                #endif
                match(target,pt);
                break;
            //the next size up, match
            } else if (node.dimensions.x <= RES * 8) {
                if(!node.getPointAt(tree->calculateIndex(target, node.orig)).isEmpty()) {
                    std::cout << "match on smallish node" << std::endl;
                    match(target,pt);
                    break;
                }
            } else {
                //check within required accuracy
                simplePoint existingPoint = node.getPointAt(tree->calculateIndex(target, node.orig));
                if(existingPoint.isEmpty()) {
                    #ifdef DEBUG
                        std::cout << "empty" << std::endl;
                    #endif
                    continue;
                }
                existingPoint = target-existingPoint;
                
                if(fabs(existingPoint.x) <= RES && fabs(existingPoint.y) <= RES && fabs(existingPoint.z) <= RES) {
                    std::cout << target.x << "," << target.y << "," << target.z << " is target" << std::endl;
                    //obviously this is a realy slow way to do this, but its only for debuging
                    std::cout << node.getPointAt(tree->calculateIndex(target, node.orig)).x << 
                        "," << node.getPointAt(tree->calculateIndex(target, node.orig)).y << 
                        "," << node.getPointAt(tree->calculateIndex(target, node.orig)).z << " is existing" << std::endl;
                    std::cout << existingPoint.x << "," << existingPoint.y << "," << existingPoint.z << " is diff" << std::endl;
                    std::cout << "match on comparision" << std::endl;
                    match(target,pt);
                    break;
                }
            }
        }
//         }
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
    #ifdef DEBUG
        std::cout << newPt << std::endl;
    #endif
    cld->push_back(newPt);
}

shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > CPURayTracer::getResult() {
    return cld;
}