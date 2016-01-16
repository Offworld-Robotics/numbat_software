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
// #include <CL/cl.hpp>


#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


const simplePoint DIMS = {50.0, 50.0, 50.0};
const ros::Duration TF_WAIT_TIME(0.1);

//quick open CL error checking function from:
//http://developer.amd.com/tools-and-sdks/opencl-zone/opencl-resources/introductory-tutorial-to-opencl/
// inline void checkErr(cl_int err, const char * name) {
//     if (err != CL_SUCCESS) {
//       std::cerr << "ERROR: " << name  << " (" << err << ")" << std::endl;
//       exit(EXIT_FAILURE);
//    }
// }

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


CameraFusionNode::CameraFusionNode() : nh(), it(nh), tfBuffer(), tfListener(tfBuffer){
    colourPub = nh.advertise<sensor_msgs::PointCloud2>(TOPIC,10,true);
    pcSub =  nh.subscribe("/pcl", 1, &CameraFusionNode::pointCloudCallback, this);
    camSub = it.subscribe("/camera0", 1, &CameraFusionNode::imageCallback, this,  image_transport::TransportHints("compressed"));
    tr = NULL;
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
    //TODO: when we start using SLAMed data we won't be able to use base_link for the stantionary frame
    geometry_msgs::TransformStamped transformStamped;
//     tfBuffer.waitForTransform(frame->header.frame_id,frame->header.stamp,pcHeader.frame_id, pcHeader.stamp, "base_link", TF_WAIT_TIME);
    try {
        ROS_INFO("%s to %s",frame->header.frame_id.c_str(), pcHeader.frame_id.c_str());
        transformStamped = tfBuffer.lookupTransform(frame->header.frame_id,frame->header.stamp,pcHeader.frame_id, pcHeader.stamp, "base_link", TF_WAIT_TIME);
        pcl::PointCloud<pcl::PointXYZ> cld;
        sensor_msgs::PointCloud2 pc2;
        tf2::doTransform(latestPointCloud, pc2, transformStamped);
        pcl::fromROSMsg<pcl::PointXYZ>(pc2, cld);
        
        //clear the octree
        if(tr) {
            delete tr;
        }
        tr = new  Octree(DIMS);
        //load the point cloud into the octree
        for (size_t i = 0; i < cld.points.size (); ++i) {
            tr->addPoint(cld.points[i]);
    //         ROS_INFO("pt.x=%f;\npt.y=%f;\npt.z=%f;\ntr->addPoint(pt)\nTEST HERE",cld.points[i].x, cld.points[i].y, cld.points[i].z);
        }
        
        tracer.setOctree(tr);
        //TODO: fix this path
//         cv::Mat image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/100sq.jpg", CV_LOAD_IMAGE_COLOR);
        const cv::Mat * image = &cv_bridge::toCvShare(frame, "bgr8")->image;
        if(!image->data) {
            std::cout << "no data" << std::endl;
        }
        tracer.loadImage(image);
        tracer.runTraces();
        shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cld2;
        cld2 =  tracer.getResult();
        sensor_msgs::PointCloud2 pcl2;
        
        pcl::toROSMsg(*cld2, pcl2);
        //use the header from the input point cloud, latter we will have to update the farme on this
        pcl2.header = latestPointCloud.header;
        std::cout << cld2->points.size() << std::endl;
        colourPub.publish(pcl2);
        
        cld2.reset();
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}


void CameraFusionNode::pointCloudCallback ( const sensor_msgs::PointCloud2::ConstPtr& pc ) {
    latestPointCloud = *pc;
    pcHeader = pc->header;
}

void CameraFusionNode::spin() {
    
    while(ros::ok()) {
        
        ros::spinOnce();
    }
}
