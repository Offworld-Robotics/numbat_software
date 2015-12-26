/*
 * Date Started: 26/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Abstract interface for ray tracing (built this way so we can easily switch to OpenCL)
 */
#ifndef RAY_TRACER_H
#define RAY_TRACER_H
#include "owr_3d_fusion/Octree.h"
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#define RES 0.0001

class RayTracer {
    
    public:
        RayTracer();
        ~RayTracer();
        
        //set the octree for this to use
        void setOctree(Octree octTree);
        //set the image to use
        void loadImage(cv::Mat image);
        //run the task, return the point cloud
        virtual void runTraces() = 0;
        //get the result of the task
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getResult();
        
        
    private:
        //the current octree
        Octree tree;
        cv::Mat image;
    
};

#endif //RAY_TRACER_H