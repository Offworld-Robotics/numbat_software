#include "owr_3d_fusion/Octree.h"
#include "owr_3d_fusion/CLRayTracer.hpp"
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gtest/gtest.h>
#include <boost/concept_check.hpp>


TEST(CLRayTraceTest, testConstructor) {
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
    
    CLRayTracer tracer;
    tracer.setOctree(&oct);
    //int rows, int cols, int type, void* data
    char a[1024] = {'\n'};
    cv::Mat testMat(4,4, CV_8UC(3),a);
    tracer.loadImage(&testMat);
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
    CLRayTracer tracer2;
    tracer2.setOctree(&oct);
    //TODO: fix this path
    cv::Mat image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/10sq.jpg", CV_LOAD_IMAGE_COLOR);
    if(!image.data) {
        std::cout << "no data" << std::endl;
    }
    tracer2.loadImage(&image);
    tracer2.runTraces();
    cld =  tracer2.getResult();
    for (size_t i = 0; i < cld->points.size (); ++i) {
        std::cout <<  cld->points[i].x << "," <<  cld->points[i].y << ","
            <<  cld->points[i].z << "," << std::endl;
        std::cout << "RGB:" <<  (uint8_t) cld->points[i].r << "," << (uint8_t) cld->points[i].g << ","
            << (uint8_t) cld->points[i].b << "," << std::endl;    
    }
    std::cout << cld->points.size() << std::endl;
    EXPECT_EQ(true, cld->points.size() > 0);
    image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/100sq.jpg", CV_LOAD_IMAGE_COLOR);
    if(!image.data) {
        std::cout << "no data" << std::endl;
    }
    tracer2.loadImage(&image);
    tracer2.runTraces();
    cld =  tracer2.getResult();
    for (size_t i = 0; i < cld->points.size (); ++i) {
        std::cout <<  cld->points[i].x << "," <<  cld->points[i].y << ","
            <<  cld->points[i].z << "," << std::endl;
        std::cout << "RGB:" <<  (uint8_t) cld->points[i].r << "," << (uint8_t) cld->points[i].g << ","
            << (uint8_t) cld->points[i].b << "," << std::endl;    
    }
    std::cout << cld->points.size() << std::endl;
    EXPECT_EQ(true, cld->points.size() > 0);
    
}

TEST(OctreeTest, testLargeTree) {
    pcl::PointXYZ dims;
    dims.x = 50.0;
    dims.y = 50.0;
    dims.z = 50.0;
    Octree oct(dims);
    const simplePoint empty = {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
    };
    std::cout << "construction done" << std::endl;
    pcl::PointXYZ pt;
    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = 0.0;
    //check we get empty
    simplePoint orig = {0.0,0.0,0.0};
    octNode rt = oct.getNode(orig);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    EXPECT_EQ(false,rt.hasChildren());
    pt.x=-3.637557;
    pt.y=-8.206607;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.669668;
    pt.y=-8.184834;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.710701;
    pt.y=-8.181890;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.750910;
    pt.y=-8.176934;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.793341;
    pt.y=-8.176276;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.830973;
    pt.y=-8.165532;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.862339;
    pt.y=-8.142081;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.905277;
    pt.y=-8.141726;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.956913;
    pt.y=-8.158076;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-3.993657;
    pt.y=-8.144788;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.040060;
    pt.y=-8.149957;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.076806;
    pt.y=-8.136287;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.111165;
    pt.y=-8.118032;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.153017;
    pt.y=-8.113722;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.199205;
    pt.y=-8.117107;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.233479;
    pt.y=-8.098320;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.261367;
    pt.y=-8.068014;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.292005;
    pt.y=-8.042850;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.319060;
    pt.y=-8.011475;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.352871;
    pt.y=-7.992168;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.433598;
    pt.y=-8.054121;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.467594;
    pt.y=-8.034315;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.499981;
    pt.y=-8.011777;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.525100;
    pt.y=-7.977073;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.567514;
    pt.y=-7.971442;
    pt.z=-0.100021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.605817;
    pt.y=-7.958748;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.659250;
    pt.y=-7.970594;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.692880;
    pt.y=-7.949808;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.721118;
    pt.y=-7.920394;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.729448;
    pt.y=-7.859591;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.787846;
    pt.y=-7.878139;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.795645;
    pt.y=-7.817266;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.854202;
    pt.y=-7.835306;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.895463;
    pt.y=-7.826217;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.928461;
    pt.y=-7.804405;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.952472;
    pt.y=-7.769145;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-4.985220;
    pt.y=-7.747085;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.030252;
    pt.y=-7.743068;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.075913;
    pt.y=-7.739625;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.119360;
    pt.y=-7.732640;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.162272;
    pt.y=-7.724600;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.194823;
    pt.y=-7.701628;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.227274;
    pt.y=-7.678515;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.256117;
    pt.y=-7.650390;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.295397;
    pt.y=-7.636719;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.339391;
    pt.y=-7.629298;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.371525;
    pt.y=-7.605556;
    pt.z=-0.100020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.398169;
    pt.y=-7.574462;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.430063;
    pt.y=-7.550466;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.499986;
    pt.y=-7.576479;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.531888;
    pt.y=-7.552040;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.568580;
    pt.y=-7.533785;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.613225;
    pt.y=-7.525596;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.653571;
    pt.y=-7.511659;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.687056;
    pt.y=-7.488898;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.764893;
    pt.y=-7.521356;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.797181;
    pt.y=-7.496540;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.828732;
    pt.y=-7.470807;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.860169;
    pt.y=-7.444937;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.895970;
    pt.y=-7.424312;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-5.967058;
    pt.y=-7.445645;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.001616;
    pt.y=-7.422990;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.032844;
    pt.y=-7.396367;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.050901;
    pt.y=-7.354459;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.081830;
    pt.y=-7.327623;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.112641;
    pt.y=-7.300653;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.143997;
    pt.y=-7.274298;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.167251;
    pt.y=-7.238851;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.297375;
    pt.y=-7.322240;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.369175;
    pt.y=-7.339484;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.423006;
    pt.y=-7.336329;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.430773;
    pt.y=-7.282907;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.508444;
    pt.y=-7.304880;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.538468;
    pt.y=-7.275324;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.569736;
    pt.y=-7.247091;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.611257;
    pt.y=-7.229555;
    pt.z=-0.100019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.641636;
    pt.y=-7.200281;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.675376;
    pt.y=-7.174456;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.664862;
    pt.y=-7.103534;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.694691;
    pt.y=-7.074028;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.765404;
    pt.y=-7.085405;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.859069;
    pt.y=-7.118822;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-6.888964;
    pt.y=-7.088469;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.142318;
    pt.y=-6.975259;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.194348;
    pt.y=-6.964717;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.223568;
    pt.y=-6.932906;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.238585;
    pt.y=-6.888193;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.367052;
    pt.y=-6.945854;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.408124;
    pt.y=-6.923944;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.437165;
    pt.y=-6.891202;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.449522;
    pt.y=-6.843829;
    pt.z=-0.100018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.466892;
    pt.y=-6.801067;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.495395;
    pt.y=-6.768070;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.523756;
    pt.y=-6.734951;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.545864;
    pt.y=-6.696538;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.637494;
    pt.y=-6.716551;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.766346;
    pt.y=-6.766579;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.805500;
    pt.y=-6.741183;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.833740;
    pt.y=-6.706712;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.861829;
    pt.y=-6.672118;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.882749;
    pt.y=-6.631769;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.883120;
    pt.y=-6.575175;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.871370;
    pt.y=-6.509413;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-7.898598;
    pt.y=-6.474658;
    pt.z=-0.100017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-8.008929;
    pt.y=-6.448017;
    pt.z=-0.100016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-8.035889;
    pt.y=-6.412663;
    pt.z=-0.100016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-8.062695;
    pt.y=-6.377193;
    pt.z=-0.100016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.260295;
    pt.y=-0.495377;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.240456;
    pt.y=-0.478425;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.237411;
    pt.y=-0.474069;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.238380;
    pt.y=-0.472684;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.237720;
    pt.y=-0.470126;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.237043;
    pt.y=-0.467578;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.236351;
    pt.y=-0.465041;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.237281;
    pt.y=-0.463661;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.238205;
    pt.y=-0.462278;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.238299;
    pt.y=-0.460324;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.237555;
    pt.y=-0.457811;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.238454;
    pt.y=-0.456428;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.239347;
    pt.y=-0.455040;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.236065;
    pt.y=-0.450889;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.233589;
    pt.y=-0.447320;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.249539;
    pt.y=-0.455758;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247907;
    pt.y=-0.452701;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.245411;
    pt.y=-0.449124;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.246273;
    pt.y=-0.447708;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247129;
    pt.y=-0.446289;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247979;
    pt.y=-0.444865;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247118;
    pt.y=-0.442394;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.241967;
    pt.y=-0.437340;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.242778;
    pt.y=-0.435941;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.235849;
    pt.y=-0.429936;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.234044;
    pt.y=-0.427041;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.234811;
    pt.y=-0.425677;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.233841;
    pt.y=-0.423309;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.236328;
    pt.y=-0.422939;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.239689;
    pt.y=-0.423043;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.244802;
    pt.y=-0.424098;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.263924;
    pt.y=-0.432869;
    pt.z=-0.100001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.264718;
    pt.y=-0.431376;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.265505;
    pt.y=-0.429879;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247787;
    pt.y=-0.418440;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.248518;
    pt.y=-0.417018;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.249243;
    pt.y=-0.415593;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.242866;
    pt.y=-0.410471;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.241785;
    pt.y=-0.408155;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.242472;
    pt.y=-0.406760;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.243153;
    pt.y=-0.405362;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.239354;
    pt.y=-0.401731;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.234632;
    pt.y=-0.397694;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.235274;
    pt.y=-0.396331;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.235911;
    pt.y=-0.394966;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247372;
    pt.y=-0.398764;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.258873;
    pt.y=-0.402465;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.264975;
    pt.y=-0.403533;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.271092;
    pt.y=-0.404550;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.277224;
    pt.y=-0.405518;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.277902;
    pt.y=-0.403971;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.289535;
    pt.y=-0.407301;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.292051;
    pt.y=-0.406506;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.292733;
    pt.y=-0.404895;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.295245;
    pt.y=-0.404070;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.296838;
    pt.y=-0.402836;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.300271;
    pt.y=-0.402364;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.311098;
    pt.y=-0.404927;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.311774;
    pt.y=-0.403233;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.311515;
    pt.y=-0.401162;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.312175;
    pt.y=-0.399467;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.311897;
    pt.y=-0.397403;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.306017;
    pt.y=-0.393169;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.306642;
    pt.y=-0.391499;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.307261;
    pt.y=-0.389826;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.307872;
    pt.y=-0.388150;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.311291;
    pt.y=-0.387510;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.314712;
    pt.y=-0.386843;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.312487;
    pt.y=-0.384122;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.312131;
    pt.y=-0.382090;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.312710;
    pt.y=-0.380394;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.299099;
    pt.y=-0.373812;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.289225;
    pt.y=-0.368638;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.279314;
    pt.y=-0.363552;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.272215;
    pt.y=-0.359495;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.268892;
    pt.y=-0.356737;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.253171;
    pt.y=-0.350048;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.253612;
    pt.y=-0.348611;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.252158;
    pt.y=-0.302687;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.292807;
    pt.y=-0.308201;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.293071;
    pt.y=-0.306594;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.293328;
    pt.y=-0.304986;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.281726;
    pt.y=-0.301500;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.272079;
    pt.y=-0.298421;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.261422;
    pt.y=-0.295279;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.261631;
    pt.y=-0.293809;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.261834;
    pt.y=-0.292339;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.247168;
    pt.y=-0.288845;
    pt.z=-0.100000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=-0.233469;
    pt.y=-0.285611;
    pt.z=-0.100000;
    oct.addPoint(pt);
        pt.x=3.637557;
    pt.y=-8.206607;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.669668;
    pt.y=-8.184834;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.710701;
    pt.y=-8.181890;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.750910;
    pt.y=-8.176934;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.793341;
    pt.y=-8.176276;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.830973;
    pt.y=-8.165532;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.862339;
    pt.y=-8.142081;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.905277;
    pt.y=-8.141726;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.956913;
    pt.y=-8.158076;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=3.993657;
    pt.y=-8.144788;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.040060;
    pt.y=-8.149957;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.076806;
    pt.y=-8.136287;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.111165;
    pt.y=-8.118032;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.153017;
    pt.y=-8.113722;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.199205;
    pt.y=-8.117107;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.233479;
    pt.y=-8.098320;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.261367;
    pt.y=-8.068014;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.292005;
    pt.y=-8.042850;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.319060;
    pt.y=-8.011475;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.352871;
    pt.y=-7.992168;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.433598;
    pt.y=-8.054121;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.467594;
    pt.y=-8.034315;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.499981;
    pt.y=-8.011777;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.525100;
    pt.y=-7.977073;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.567514;
    pt.y=-7.971442;
    pt.z=-0.000021;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.605817;
    pt.y=-7.958748;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.659250;
    pt.y=-7.970594;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.692880;
    pt.y=-7.949808;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.721118;
    pt.y=-7.920394;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.729448;
    pt.y=-7.859591;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.787846;
    pt.y=-7.878139;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.795645;
    pt.y=-7.817266;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.854202;
    pt.y=-7.835306;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.895463;
    pt.y=-7.826217;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.928461;
    pt.y=-7.804405;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.952472;
    pt.y=-7.769145;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=4.985220;
    pt.y=-7.747085;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.030252;
    pt.y=-7.743068;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.075913;
    pt.y=-7.739625;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.119360;
    pt.y=-7.732640;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.162272;
    pt.y=-7.724600;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.194823;
    pt.y=-7.701628;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.227274;
    pt.y=-7.678515;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.256117;
    pt.y=-7.650390;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.295397;
    pt.y=-7.636719;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.339391;
    pt.y=-7.629298;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.371525;
    pt.y=-7.605556;
    pt.z=-0.000020;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.398169;
    pt.y=-7.574462;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.430063;
    pt.y=-7.550466;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.499986;
    pt.y=-7.576479;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.531888;
    pt.y=-7.552040;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.568580;
    pt.y=-7.533785;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.613225;
    pt.y=-7.525596;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.653571;
    pt.y=-7.511659;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.687056;
    pt.y=-7.488898;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.764893;
    pt.y=-7.521356;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.797181;
    pt.y=-7.496540;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.828732;
    pt.y=-7.470807;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.860169;
    pt.y=-7.444937;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.895970;
    pt.y=-7.424312;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=5.967058;
    pt.y=-7.445645;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.001616;
    pt.y=-7.422990;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.032844;
    pt.y=-7.396367;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.050901;
    pt.y=-7.354459;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.081830;
    pt.y=-7.327623;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.112641;
    pt.y=-7.300653;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.143997;
    pt.y=-7.274298;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.167251;
    pt.y=-7.238851;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.297375;
    pt.y=-7.322240;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.369175;
    pt.y=-7.339484;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.423006;
    pt.y=-7.336329;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.430773;
    pt.y=-7.282907;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.508444;
    pt.y=-7.304880;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.538468;
    pt.y=-7.275324;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.569736;
    pt.y=-7.247091;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.611257;
    pt.y=-7.229555;
    pt.z=-0.000019;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.641636;
    pt.y=-7.200281;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.675376;
    pt.y=-7.174456;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.664862;
    pt.y=-7.103534;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.694691;
    pt.y=-7.074028;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.765404;
    pt.y=-7.085405;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.859069;
    pt.y=-7.118822;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=6.888964;
    pt.y=-7.088469;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.142318;
    pt.y=-6.975259;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.194348;
    pt.y=-6.964717;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.223568;
    pt.y=-6.932906;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.238585;
    pt.y=-6.888193;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.367052;
    pt.y=-6.945854;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.408124;
    pt.y=-6.923944;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.437165;
    pt.y=-6.891202;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.449522;
    pt.y=-6.843829;
    pt.z=-0.000018;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.466892;
    pt.y=-6.801067;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.495395;
    pt.y=-6.768070;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.523756;
    pt.y=-6.734951;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.545864;
    pt.y=-6.696538;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.637494;
    pt.y=-6.716551;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.766346;
    pt.y=-6.766579;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.805500;
    pt.y=-6.741183;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.833740;
    pt.y=-6.706712;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.861829;
    pt.y=-6.672118;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.882749;
    pt.y=-6.631769;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.883120;
    pt.y=-6.575175;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.871370;
    pt.y=-6.509413;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=7.898598;
    pt.y=-6.474658;
    pt.z=-0.000017;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=8.008929;
    pt.y=-6.448017;
    pt.z=-0.000016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=8.035889;
    pt.y=-6.412663;
    pt.z=-0.000016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=8.062695;
    pt.y=-6.377193;
    pt.z=-0.000016;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.260295;
    pt.y=-0.495377;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.240456;
    pt.y=-0.478425;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.237411;
    pt.y=-0.474069;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.238380;
    pt.y=-0.472684;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.237720;
    pt.y=-0.470126;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.237043;
    pt.y=-0.467578;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.236351;
    pt.y=-0.465041;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.237281;
    pt.y=-0.463661;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.238205;
    pt.y=-0.462278;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.238299;
    pt.y=-0.460324;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.237555;
    pt.y=-0.457811;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.238454;
    pt.y=-0.456428;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.239347;
    pt.y=-0.455040;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.236065;
    pt.y=-0.450889;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.233589;
    pt.y=-0.447320;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.249539;
    pt.y=-0.455758;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247907;
    pt.y=-0.452701;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.245411;
    pt.y=-0.449124;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.246273;
    pt.y=-0.447708;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247129;
    pt.y=-0.446289;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247979;
    pt.y=-0.444865;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247118;
    pt.y=-0.442394;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.241967;
    pt.y=-0.437340;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.242778;
    pt.y=-0.435941;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.235849;
    pt.y=-0.429936;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.234044;
    pt.y=-0.427041;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.234811;
    pt.y=-0.425677;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.233841;
    pt.y=-0.423309;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.236328;
    pt.y=-0.422939;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.239689;
    pt.y=-0.423043;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.244802;
    pt.y=-0.424098;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.263924;
    pt.y=-0.432869;
    pt.z=-0.000001;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.264718;
    pt.y=-0.431376;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.265505;
    pt.y=-0.429879;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247787;
    pt.y=-0.418440;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.248518;
    pt.y=-0.417018;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.249243;
    pt.y=-0.415593;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.242866;
    pt.y=-0.410471;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.241785;
    pt.y=-0.408155;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.242472;
    pt.y=-0.406760;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.243153;
    pt.y=-0.405362;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.239354;
    pt.y=-0.401731;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.234632;
    pt.y=-0.397694;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.235274;
    pt.y=-0.396331;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.235911;
    pt.y=-0.394966;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247372;
    pt.y=-0.398764;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.258873;
    pt.y=-0.402465;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.264975;
    pt.y=-0.403533;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.271092;
    pt.y=-0.404550;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.277224;
    pt.y=-0.405518;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.277902;
    pt.y=-0.403971;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.289535;
    pt.y=-0.407301;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.292051;
    pt.y=-0.406506;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.292733;
    pt.y=-0.404895;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.295245;
    pt.y=-0.404070;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.296838;
    pt.y=-0.402836;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.300271;
    pt.y=-0.402364;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.311098;
    pt.y=-0.404927;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.311774;
    pt.y=-0.403233;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.311515;
    pt.y=-0.401162;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.312175;
    pt.y=-0.399467;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.311897;
    pt.y=-0.397403;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.306017;
    pt.y=-0.393169;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.306642;
    pt.y=-0.391499;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.307261;
    pt.y=-0.389826;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.307872;
    pt.y=-0.388150;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.311291;
    pt.y=-0.387510;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.314712;
    pt.y=-0.386843;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.312487;
    pt.y=-0.384122;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.312131;
    pt.y=-0.382090;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.312710;
    pt.y=-0.380394;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.299099;
    pt.y=-0.373812;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.289225;
    pt.y=-0.368638;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.279314;
    pt.y=-0.363552;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.272215;
    pt.y=-0.359495;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.268892;
    pt.y=-0.356737;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.253171;
    pt.y=-0.350048;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.253612;
    pt.y=-0.348611;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.252158;
    pt.y=-0.302687;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.292807;
    pt.y=-0.308201;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.293071;
    pt.y=-0.306594;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.293328;
    pt.y=-0.304986;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.281726;
    pt.y=-0.301500;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.272079;
    pt.y=-0.298421;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.261422;
    pt.y=-0.295279;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.261631;
    pt.y=-0.293809;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.261834;
    pt.y=-0.292339;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.247168;
    pt.y=-0.288845;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
     pt.x=0.233469;
    pt.y=-0.285611;
    pt.z=-0.000000;
    oct.addPoint(pt);
    //TEST HERE
    std::cout << "Done Load" << std::endl;
    CLRayTracer tracer2;
    tracer2.setOctree(&oct);
    //TODO: fix this path
    cv::Mat image = cv::imread("/home/ros/owr_software/rover/src/owr_3d_fusion/test/10sq.jpg", CV_LOAD_IMAGE_COLOR);
    if(!image.data) {
        std::cout << "no data" << std::endl;
    }
    tracer2.loadImage(&image);
    tracer2.runTraces();
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cld =  tracer2.getResult();
    for (size_t i = 0; i < cld->points.size (); ++i) {
        std::cout <<  cld->points[i].x << "," <<  cld->points[i].y << ","
            <<  cld->points[i].z << "," << std::endl;
        std::cout << "RGB:" <<  (uint8_t) cld->points[i].r << "," << (uint8_t) cld->points[i].g << ","
            << (uint8_t) cld->points[i].b << "," << std::endl;    
    }
    std::cout << cld->points.size() << std::endl;
    EXPECT_EQ(true, cld->points.size() > 0);
}


int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}