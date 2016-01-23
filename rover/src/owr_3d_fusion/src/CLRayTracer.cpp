/*
 * Date Started: 19/01/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: A CL based ray tracer
 */

#include "owr_3d_fusion/CLRayTracer.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>
#include <math.h>
#include <malloc.h>

#include "owr_3d_fusion/logitechC920.h"


#define IMG_MAX_WIDTH 1920
#define IMG_MAX_HEIGHT 1080
#define MAX_KERNEL_SIZE 20000
// #define DEBUG

inline void checkErr(cl_int err, const char * name) {
    if (err != CL_SUCCESS) {
      ROS_ERROR("%s: (%d)",name,err);
      exit(EXIT_FAILURE);
   }
}

CLRayTracer::CLRayTracer() : RayTracer(), cld(new pcl::PointCloud<pcl::PointXYZRGB> ()) {
    //this code based on opencl tutorial at 
    //http://simpleopencl.blogspot.com.au/2013/06/tutorial-simple-start-with-opencl-and-c.html
    
    //TODO: prioritise intel GPU platform
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if(all_platforms.size()==0){
        ROS_ERROR(" No platforms found!");
        exit(1);
    }
    cl::Platform default_platform=all_platforms[0];
    
    ROS_INFO("Using platform: %s\n",default_platform.getInfo<CL_PLATFORM_NAME>().c_str());
    
    std::vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    if(all_devices.size()==0){
        ROS_ERROR(" No devices found. Check OpenCL installation!");
        exit(1);
    }
    cl::Device default_device=all_devices[0];
    ROS_INFO("Using device: %s",default_device.getInfo<CL_DEVICE_NAME>().c_str());
    context = new cl::Context(default_device);
    
     
    
    //load the kernel file
    std::string path = ros::package::getPath("owr_3d_fusion");
    char *sourceStr;
    size_t sourceSize;
    FILE *fp= fopen((path + "/cl/rayTrace.cl").c_str(), "r");
    
    if(!fp) {
        ROS_ERROR("File not found: %s", (path + "/cl/rayTrace.cl").c_str());
        ROS_ERROR("Error: %d (%s)\n", errno, strerror(errno));
        exit(1);
    }
    sourceStr = (char*)malloc(MAX_KERNEL_SIZE);
    sourceSize = fread(sourceStr, 1, MAX_KERNEL_SIZE, fp);
    fclose(fp);
    
    cl::Program::Sources sources;
    sources.push_back({sourceStr, sourceSize});
    //TODO: load kernel from file
    device.push_back(default_device);
    cl::Program program(*context,sources);
    if(program.build(device)!=CL_SUCCESS){
        ROS_ERROR(" Error building: %s",program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device).c_str());
        exit(1);
    }

    cl_int err;    
    rayTrace = new cl::Kernel(program, "rayTrace", &err);
    defaultDevice = default_device;
    
    checkErr(err, "cl::Kernel");
    
    
    //NOTE: we can't create the image buffer here because we don't know how big it is going to be
    result = new cl::Buffer(*context, CL_MEM_READ_WRITE,sizeof(float)*8*IMG_MAX_WIDTH*IMG_MAX_HEIGHT);
    
    //NOTE: this is a very large buffer, intel recomends doing it this way rather then copying it around
    //se page 31 of the intel guide
    int cachelineSize = defaultDevice.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);
    checkErr(err, "Get GLOBAL_MEM_CACHELINE");
    size_t arraySizeAligned = cachelineSize*(1+(sizeof(octNodeCL)*8*HASH_MAP_SIZE-1)/cachelineSize);//aligned
    checkErr(err, "load flatTree");
    //void* inputArray;
    /*err = posix_memalign(&inputArray, 4096, arraySizeAligned);
    if(!result) {
        ROS_ERROR("failed to allocate flatTree. Error %d", err);
        exit(0);
    }*/
    treeBuffer =  new cl::Buffer(*context, CL_MEM_READ_ONLY| CL_MEM_ALLOC_HOST_PTR,arraySizeAligned);
    //treeBuffer = new cl::Buffer(*context,CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR,arraySizeAligned,inputArray,&err);
    //treeBuffer = new cl::Buffer(*context,CL_MEM_READ_ONLY |CL_MEM_USE_HOST_PTR ,arraySizeAligned,inputArray,&err);
    //flatTree = (octNodeCL *) inputArray;
    dimsBuffer = new cl::Buffer(*context, CL_MEM_READ_ONLY,sizeof(cl_float4));
    

    queue = new cl::CommandQueue(*context,default_device);
    flatTree = (octNodeCL *) queue->enqueueMapBuffer(*treeBuffer, CL_TRUE, CL_MAP_READ,0, arraySizeAligned,NULL,NULL, &err);
    checkErr(err, "map buffer");
    
}

CLRayTracer::~CLRayTracer() {
    cld.reset();
    queue->enqueueUnmapMemObject(*treeBuffer, flatTree, NULL,NULL);
    delete queue;
    delete treeBuffer;
    
    //free(flatTree);
    delete dimsBuffer;
}

void CLRayTracer::loadImage(const cv::Mat * image) {
    RayTracer::loadImage(image);
    if(imgBuffer) {
        delete imgBuffer;
    }
    /*if(rayTrace) {
        delete rayTrace;
    }
    rayTrace = new cl::KernelFunctor(cl::Kernel(program,"rayTrace"),queue,cl::NullRange,cl::NDRange(image->rows,image->cols),cl::NullRange);*/
    const size_t imgSize = sizeof(cl_uchar3)*image->rows*image->cols;
    imgBuffer = new cl::Buffer(*context, CL_MEM_READ_ONLY,imgSize);
    cl_float4 dims;
    dims.y = image->cols;
    dims.z = image->rows;
    cl::Event dimsWriteEvent;
    std::vector<cl::Event> events;
    queue->enqueueWriteBuffer(*dimsBuffer, CL_TRUE, 0, sizeof(dims), &dims,&events, &dimsWriteEvent);
    queue->enqueueWriteBuffer(*imgBuffer, CL_TRUE, 0, imgSize, (cl_uchar3*)image->data);
    //we need to make sure dims are written before it goes out of scope
    dimsWriteEvent.wait();
}


void CLRayTracer::runTraces() {
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
    size_t wgSize;
    
    rayTrace->getWorkGroupInfo(defaultDevice, CL_KERNEL_WORK_GROUP_SIZE, &wgSize);
    ROS_INFO("Max Workgroups %lu", wgSize);
     
    ROS_INFO("Setting args");
    cl_int err;
    err = rayTrace->setArg(0, *result);
    checkErr(err, "Kernel::setArg()");
    err = rayTrace->setArg(1, *imgBuffer);
    checkErr(err, "Kernel::setArg()");
    err = rayTrace->setArg(2, *treeBuffer);
    checkErr(err, "Kernel::setArg()");
    err = rayTrace->setArg(3, *dimsBuffer);
    checkErr(err, "Kernel::setArg()");

    ROS_INFO("Write flatTree");    
    flatTree = tree->getFlatTree(flatTree);
    int cachelineSize = defaultDevice.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);
    checkErr(err, "Get GLOBAL_MEM_CACHELINE");
    size_t arraySizeAligned = cachelineSize*(1+(sizeof(octNodeCL)*8*HASH_MAP_SIZE-1)/cachelineSize);//aligned
    //queue->enqueueMapBuffer(*treeBuffer, CL_TRUE, CL_MAP_READ,0, arraySizeAligned,NULL,NULL, &err);
    ROS_INFO("FlatTree[1], dims %f, locCode %lu", flatTree[1].dimensions, flatTree[1].locCode);
    //std::vector<cl::Event> writeEvents;
    //cl::Event writeEvent;
    //queue->enqueueWriteBuffer(*treeBuffer, CL_TRUE, 0, sizeof(octNodeCL)*8*HASH_MAP_SIZE, flatTree, &writeEvents, &writeEvent);

    std::vector<cl::Event> events;
    int xOffset, yOffset;
    int xGroups = (image->rows/wgSize + 1);
    int yGroups = (image->rows/wgSize + 1);
    ROS_INFO("Begin Spawning");
    for(xOffset = 0; xOffset < xGroups; xOffset++) {
        int xRange = wgSize;
        if(image->rows - xRange*xOffset < xRange) {
            xRange = image->rows % xRange;
        }
        for(yOffset = 0; yOffset < yGroups; yOffset++) {
            
            int yRange = wgSize;
            if(image->cols - yRange*yOffset < yRange) {
                yRange = image->cols % yRange;
            } 
            ROS_INFO("Spawning with workers %d, %d", xRange, yRange);
            cl::Event event;
            events.push_back(event);
            err = queue->enqueueNDRangeKernel(
                *rayTrace,
                cl::NDRange(xOffset,yOffset),
                cl::NDRange(xRange,yRange),
                cl::NDRange(1,1),
                NULL,
                &events.back()
            );
            checkErr(err, "running kernel");
        }
    }
    cl_float8 * results = (cl_float8 *) malloc(sizeof(float)*8*IMG_MAX_WIDTH*IMG_MAX_HEIGHT);
//     event.wait();
    cl::Event readEvent;
    err =queue->enqueueReadBuffer(*result,CL_TRUE,0,sizeof(float)*8*IMG_MAX_WIDTH*IMG_MAX_HEIGHT,results,&events,&readEvent);
    checkErr(err, "enque read");
    ROS_INFO("Finished Reading");
    readEvent.wait();
    
    cld.reset();
    shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > newCld(new pcl::PointCloud<pcl::PointXYZRGB> ());
    cld = newCld;
    pcl::PointXYZRGB newPt;
    int i;
    for(i = 0; i< IMG_MAX_WIDTH*IMG_MAX_HEIGHT; i++) {
        if(results[i].x != FLOAT_INF) {
            newPt.x = results[i].x;
            newPt.y = results[i].y;
            newPt.z = results[i].z;
            newPt.r = results[i].s3;
            newPt.g = results[i].s4;
            newPt.b = results[i].s5;
            #ifdef DEBUG
                std::cout << newPt << std::endl;
            #endif
            cld->push_back(newPt);
        }
    }

    free(results);
    
    
}

void CLRayTracer::match ( simplePoint pt, cv::Vec3i pixel ) {
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

shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > CLRayTracer::getResult() {
    return cld;
}
