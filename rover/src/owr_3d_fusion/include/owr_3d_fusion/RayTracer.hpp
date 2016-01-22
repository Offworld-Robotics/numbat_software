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
#include <boost/shared_ptr.hpp>
#define RES 0.01 //(1cm)
#define TRACE_RANGE 60


using namespace boost;

class RayTracer {
    
    public:
        RayTracer();
        ~RayTracer();
        
        //set the octree for this to use
        void setOctree(Octree * octTree);
        //set the image to use
        virtual void loadImage(const cv::Mat * image);
        //run the task, return the point cloud
        virtual void runTraces() = 0;
        //get the result of the task
        virtual shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getResult() = 0;
        
        
    protected:
        //the current octree
        Octree * tree = NULL;
        const cv::Mat * image;
    
};

#endif //RAY_TRACER_H
