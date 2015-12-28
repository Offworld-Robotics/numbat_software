#include "owr_3d_fusion/Octree.h"
#include "owr_3d_fusion/CPURayTracer.hpp"
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gtest/gtest.h>
#include <boost/concept_check.hpp>


TEST(CPURayTraceTest, testConstructor) {
    pcl::PointXYZ pt;
    pt.x = 50.0;
    pt.y = 50.0;
    pt.z = 50.0;
    Octree oct(pt);
    pcl::PointXYZ pt1(0.0,0.0,10.0);
    pcl::PointXYZ pt2(0.0,0.0,-10.0);
    pcl::PointXYZ pt3(0.0,10.0,0.0);
    pcl::PointXYZ pt4(0.0,-10.0,0.0);
    pcl::PointXYZ pt5(10.0,0.0,0.0);
    pcl::PointXYZ pt6(-10.0,0.0,0.0);
    oct.addPoint(pt1);
    oct.addPoint(pt2);
    oct.addPoint(pt3);
    oct.addPoint(pt4);
    oct.addPoint(pt5);
    oct.addPoint(pt6);
    
    CPURayTracer tracer;
    tracer.setOctree(&oct);
    //int rows, int cols, int type, void* data
    char a[1024] = {'\n'};
    cv::Mat testMat(4,4, CV_8UC(3),a);
    tracer.loadImage(testMat);
    tracer.runTraces();
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cld =  tracer.getResult();
    for (size_t i = 0; i < cld->points.size (); ++i) {
        std::cout <<  cld->points[i].x << "," <<  cld->points[i].y << ","
            <<  cld->points[i].z << "," << std::endl;
        std::cout << "RGB:" <<  (uint8_t) cld->points[i].r << "," << (uint8_t) cld->points[i].g << ","
            << (uint8_t) cld->points[i].b << "," << std::endl;    
    }
    //TODO: write none visual tests here
    std::cout << "actual image" << std::endl;
    CPURayTracer tracer2;
    tracer2.setOctree(&oct);
    //TODO: fix this path
    cv::Mat image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/test.jpg", CV_LOAD_IMAGE_COLOR);
    if(!image.data) {
        std::cout << "no data" << std::endl;
    }
    tracer2.loadImage(image);
    tracer2.runTraces();
    cld =  tracer2.getResult();
    for (size_t i = 0; i < cld->points.size (); ++i) {
        std::cout <<  cld->points[i].x << "," <<  cld->points[i].y << ","
            <<  cld->points[i].z << "," << std::endl;
        std::cout << "RGB:" <<  (uint8_t) cld->points[i].r << "," << (uint8_t) cld->points[i].g << ","
            << (uint8_t) cld->points[i].b << "," << std::endl;    
    }
    std::cout << cld->points.size() << std::endl;
}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}