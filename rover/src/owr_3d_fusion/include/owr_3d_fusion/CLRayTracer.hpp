/*
 * Date Started: 19/01/16
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: An OpenCL based ray tracer
 */
#ifndef CL_RAY_TRACER_H
#define CL_RAY_TRACER_H
#include "RayTracer.hpp"
#include "Octree.h"
#include <CL/cl.hpp>
class CLRayTracer: public RayTracer {  
    public:
        void runTraces();
        CLRayTracer() ;
        ~CLRayTracer();
        virtual shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getResult();
        virtual void loadImage(const cv::Mat * image);
    private:
        void match(simplePoint pt, cv::Vec3i pixel);
        shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > cld;
        
        cl::Context * context;
        cl::Buffer * imgBuffer = NULL;
        cl::Buffer * treeBuffer = NULL;
        cl::Buffer * result = NULL;
        cl::Buffer * dimsBuffer = NULL;
        cl::CommandQueue * queue;
        std::vector<cl::Device> device;
        cl::Kernel * rayTrace = NULL; 
};
#endif
