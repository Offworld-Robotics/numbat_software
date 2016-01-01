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
#include <utility>
// #define __NO_STD_VECTOR // Use cl::vector instead of STL version //This is apparently depricated
#include <CL/cl.hpp>


#include </opt/ros/indigo/include/pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

const simplePoint DIMS = {50.0, 50.0, 50.0};

//quick open CL error checking function from:
//http://developer.amd.com/tools-and-sdks/opencl-zone/opencl-resources/introductory-tutorial-to-opencl/
inline void checkErr(cl_int err, const char * name) {
    if (err != CL_SUCCESS) {
      std::cerr << "ERROR: " << name  << " (" << err << ")" << std::endl;
      exit(EXIT_FAILURE);
   }
}

int main(int argc, char ** argv) {
    ros::init(argc,argv,"camera_fusion_node");
    CameraFusionNode node;
    node.spin();
    
//    cl_int err;
//    std::vector< cl::Platform > platformList;
//    cl::Platform::get(&platformList);
//    checkErr(platformList.size()!=0 ? CL_SUCCESS : -1, "cl::Platform::get");
//    std::cerr << "Platform number is: " << platformList.size() << std::endl;std::string platformVendor;
//    platformList[0].getInfo((cl_platform_info)CL_PLATFORM_VENDOR, &platformVendor);
//    std::cerr << "Platform is by: " << platformVendor << "\n";
//    cl_context_properties cprops[3] = {CL_CONTEXT_PLATFORM, (cl_context_properties)(platformList[0])(), 0};cl::Context context(
//       CL_DEVICE_TYPE_CPU,
//       cprops,
//       NULL,
//       NULL,
//       &err);
//    checkErr(err, "Context::Context()");
}


CameraFusionNode::CameraFusionNode() : tr(DIMS) {
    colourPub = nh.advertise<sensor_msgs::PointCloud2>(TOPIC,10,true);
    pcSub =  nh.subscribe("/pcl", 1000, &CameraFusionNode::pointCloudCallback, this);
    
}

CameraFusionNode::~CameraFusionNode() {

}

sensor_msgs::PointCloud2 CameraFusionNode::doColouring ( pcl::PointCloud< pcl::PointXYZRGB > pc, sensor_msgs::Image img ) {
    return sensor_msgs::PointCloud2();
}

pcl::PointCloud< pcl::PointXYZRGB > CameraFusionNode::getLatestPointCloud() {
    
    return pcl::PointCloud<pcl::PointXYZRGB>();
}

void CameraFusionNode::imageCallback ( const sensor_msgs::Image::ConstPtr& frame ) {
    pcl::PointXYZ dims;
    dims.x = 50.0;
    dims.y = 50.0;
    dims.z = 50.0;
    Octree oct(dims);
//     pcl::PointCloud<pcl::PointXYZRGB> pc = getLatestPointCloud();
//     int x,y;
//     float deltaX, deltaY;
//     //we asume a 15m^2 grid. That can grow as needed
//     
//     //pc.size()
//     //calc this here so it only does the math once, #defines will run this many time
//     const float focalLengthPx = (PIXEL_TO_M_RATIO/FOCAL_LENGTH_M);
//     for(x = 0; x < frame->width; x++) {
//         deltaX = tanh(x*focalLengthPx);
//         for(y = 0; y < frame->height; y++) {
//             deltaY = tanh(y*focalLengthPx);
//             //gradient of z is the minimum resolution on the z axis of the point cloud
//             float px, py, pz;
//             float dist = 0;
// #define RES 1
//             //TODO: 
//             for(dist = 0;dist < 100; dist+=RES) {
//                 px = deltaX * RES;
//                 py = deltaY * RES;
//                 pz = dist;
//                 pc.at(px,py,px);
//             }
//         }
//     }
}

void CameraFusionNode::pointCloudCallback ( const sensor_msgs::PointCloud2::ConstPtr& pc ) {
    pcl::PointCloud<pcl::PointXYZ> cld;
    sensor_msgs::PointCloud2 pc2 = *pc;
    pcl::fromROSMsg<pcl::PointXYZ>(pc2, cld);
    
    //TODO: clear the octree
    
    //load the point cloud into the octree
    for (size_t i = 0; i < cld.points.size (); ++i) {
        tr.addPoint(cld.points[i]);
    }
    
    tracer.setOctree(&tr);
    //TODO: fix this path
    cv::Mat image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/test.jpg", CV_LOAD_IMAGE_COLOR);
    if(!image.data) {
        std::cout << "no data" << std::endl;
    }
    tracer.loadImage(image);
    tracer.runTraces();
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cld2;
    cld2 =  tracer.getResult();
    sensor_msgs::PointCloud2 pcl2;
    
    pcl::toROSMsg(*cld2, pcl2);
    pcl2.header.frame_id = pc->header.frame_id;
    std::cout << cld.points.size() << std::endl;
    colourPub.publish(pcl2);
}

void CameraFusionNode::spin() {
    
    while(ros::ok()) {
        
        ros::spinOnce();
    }
}
