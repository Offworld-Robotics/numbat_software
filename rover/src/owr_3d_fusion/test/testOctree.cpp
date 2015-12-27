#include "owr_3d_fusion/Octree.h"
#include <pcl/point_types.h>

#include <gtest/gtest.h>
#include <boost/concept_check.hpp>

TEST(OctreeTest, testConstructor) {
    pcl::PointXYZ pt;
    pt.x = 50.0;
    pt.y = 50.0;
    pt.z = 50.0;
    Octree oct(pt);
    
}

TEST(OctreeTest, testDimensions) {
    pcl::PointXYZ pt;
    pt.x = 50.0;
    pt.y = 50.0;
    pt.z = 50.0;
    simplePoint dims = {50.0, 50.0, 50.0};
    Octree oct(pt);
    EXPECT_EQ(dims,oct.getDimensions());
}

//inserts 9 points to test spliting, etc.
//expected outcome is no segfaults
TEST(OctreeTest, testAddNoSegfault) {
    pcl::PointXYZ dims;
    dims.x = 50.0;
    dims.y = 50.0;
    dims.z = 50.0;
    Octree oct(dims);
    std::cout << "construction done" << std::endl;
    pcl::PointXYZ pt;
    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = 0.0;
    oct.addPoint(pt);
    std::cout << "1" << std::endl;
//     EXPECT_EQ(1, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt2;
    pt2.x = 1.0;
    pt2.y = 0.0;
    pt2.z = 0.0;
    oct.addPoint(pt2);
    std::cout << "2" << std::endl;
//     EXPECT_EQ(2, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt3;
    pt3.x = 1.0;
    pt3.y = 1.0;
    pt3.z = 0.0;
    oct.addPoint(pt3);
    std::cout << "3" << std::endl;
//     EXPECT_EQ(3, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt4;
    pt4.x = 1.0;
    pt4.y = 1.0;
    pt4.z = 1.0;
    oct.addPoint(pt4);
    std::cout << "4" << std::endl;
//     EXPECT_EQ(4, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt5;
    pt5.x = 1.0;
    pt5.y = 0.0;
    pt5.z = 1.0;
    oct.addPoint(pt5);
    std::cout << "5" << std::endl;
//     EXPECT_EQ(5, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt6;
    pt6.x = 0.0;
    pt6.y = 1.0;
    pt6.z = 1.0;
    oct.addPoint(pt6);
    std::cout << "6" << std::endl;
//     EXPECT_EQ(6, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt7;
    pt7.x = 0.0;
    pt7.y = 1.0;
    pt7.z = 0.0;
    oct.addPoint(pt7);
    std::cout << "7" << std::endl;
//     EXPECT_EQ(7, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt8;
    pt8.x = 0.0;
    pt8.y = 0.0;
    pt8.z = 1.0;
    oct.addPoint(pt8);
    std::cout << "8" << std::endl;
//     EXPECT_EQ(8, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt9;
    pt9.x = 0.0;
    pt9.y = 0.0;
    pt9.z = 1.1;
    oct.addPoint(pt9);
    std::cout << "9" << std::endl;
//     EXPECT_EQ(9, oct.getNumPoints());
//     EXPECT_EQ(2, oct.getDepth());
}

//inserts 9 points to test spliting, etc.
//also tests retrival
TEST(OctreeTest, testAddAndRetrive) {
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
    std::cout << "index: " << oct.calculateIndex(rt.orig, orig) << std::endl;
    EXPECT_EQ(empty, rt.getPointAt(oct.calculateIndex(rt.orig, orig)));
    oct.addPoint(pt);
    rt = oct.getNode(pt);
    EXPECT_EQ(orig, rt.getPointAt(oct.calculateIndex(rt.orig, orig)));
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    EXPECT_EQ(true,rt.hasChildren());
    std::cout << "1" << std::endl;
//     EXPECT_EQ(1, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt2;
    pt2.x = 1.0;
    pt2.y = 0.0;
    pt2.z = 0.0;
    
    oct.addPoint(pt2);
    rt = oct.getNode(pt2);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt2 = {pt2.x,pt2.y, pt2.z};
    EXPECT_EQ(spt2, rt.getPointAt(oct.calculateIndex(spt2, rt.orig)));
    std::cout << "2" << std::endl;
//     EXPECT_EQ(2, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt3;
    pt3.x = 1.0;
    pt3.y = 1.0;
    pt3.z = 0.0;
    oct.addPoint(pt3);
    rt = oct.getNode(pt3);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt3 = {pt3.x,pt3.y, pt3.z};
    EXPECT_EQ(spt3, rt.getPointAt(oct.calculateIndex(spt3, rt.orig)));
    std::cout << "3" << std::endl;
//     EXPECT_EQ(3, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt4;
    pt4.x = 1.0;
    pt4.y = 1.0;
    pt4.z = 1.0;
    oct.addPoint(pt4);
    rt = oct.getNode(pt4);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt4 = {pt4.x,pt4.y, pt4.z};
    EXPECT_EQ(spt4, rt.getPointAt(oct.calculateIndex(spt4, rt.orig)));
    std::cout << "4" << std::endl;
//     EXPECT_EQ(4, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt5;
    pt5.x = 1.0;
    pt5.y = 0.0;
    pt5.z = 1.0;
    oct.addPoint(pt5);
    rt = oct.getNode(pt5);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt5 = {pt5.x,pt5.y, pt5.z};
    EXPECT_EQ(spt5, rt.getPointAt(oct.calculateIndex(spt5, rt.orig)));
    std::cout << "5" << std::endl;
//     EXPECT_EQ(5, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt6;
    pt6.x = 0.0;
    pt6.y = 1.0;
    pt6.z = 1.0;
    oct.addPoint(pt6);
    rt = oct.getNode(pt6);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt6 = {pt6.x,pt6.y, pt6.z};
    EXPECT_EQ(spt6, rt.getPointAt(oct.calculateIndex(spt6, rt.orig)));
    std::cout << "6" << std::endl;
//     EXPECT_EQ(6, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt7;
    pt7.x = 0.0;
    pt7.y = 1.0;
    pt7.z = 0.0;
    oct.addPoint(pt7);
    rt = oct.getNode(pt7);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt7 = {pt7.x,pt7.y, pt7.z};
    EXPECT_EQ(spt7, rt.getPointAt(oct.calculateIndex(spt7, rt.orig)));
    std::cout << "7" << std::endl;
//     EXPECT_EQ(7, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt8;
    pt8.x = 0.0;
    pt8.y = 0.0;
    pt8.z = 1.0;
    oct.addPoint(pt8);
    rt = oct.getNode(pt8);
    //should be a leaf
    EXPECT_EQ(true,rt.isLeaf());
    //check retrival
    simplePoint spt8 = {pt8.x,pt8.y, pt8.z};
    EXPECT_EQ(spt8, rt.getPointAt(oct.calculateIndex(spt8, rt.orig)));
    std::cout << "8" << std::endl;
//     EXPECT_EQ(8, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
    pcl::PointXYZ pt9;
    pt9.x = 0.0;
    pt9.y = 0.0;
    pt9.z = 1.1;
    oct.addPoint(pt9);
    rt = oct.getNode(pt9);
    //should be a leaf
//     EXPECT_EQ(false,rt.isLeaf());
    //check retrival
    simplePoint spt9 = {pt9.x,pt9.y, pt9.z};
    EXPECT_EQ(spt9, rt.getPointAt(oct.calculateIndex(spt9, rt.orig)));
    std::cout << "9" << std::endl;
//     EXPECT_EQ(9, oct.getNumPoints());
//     EXPECT_EQ(2, oct.getDepth());
}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}