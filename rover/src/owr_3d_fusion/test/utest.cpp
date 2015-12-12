#include "owr_3d_fusion/Octree.h"
#include <pcl/point_types.h>

#include <gtest/gtest.h>
#include <boost/concept_check.hpp>

TEST(OctreeTest, testConstructor) {
    Octree oct;
    
}

TEST(OctreeTest, testAdd) {
//     Octree oct;
//     pcl::PointXYZ pt;
//     pt.x = 0.0;
//     pt.y = 0.0;
//     pt.z = 0.0;
//     oct.addPoint(&pt);
//     EXPECT_EQ(1, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt2;
//     pt2.x = 1.0;
//     pt2.y = 0.0;
//     pt2.z = 0.0;
//     oct.addPoint(&pt2);
//     EXPECT_EQ(2, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt3;
//     pt3.x = 1.0;
//     pt3.y = 1.0;
//     pt3.z = 0.0;
//     oct.addPoint(&pt3);
//     EXPECT_EQ(3, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt4;
//     pt4.x = 1.0;
//     pt4.y = 1.0;
//     pt4.z = 1.0;
//     oct.addPoint(&pt4);
//     EXPECT_EQ(4, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt5;
//     pt5.x = 1.0;
//     pt5.y = 0.0;
//     pt5.z = 1.0;
//     oct.addPoint(&pt5);
//     EXPECT_EQ(5, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt6;
//     pt6.x = 0.0;
//     pt6.y = 1.0;
//     pt6.z = 1.0;
//     oct.addPoint(&pt6);
//     EXPECT_EQ(6, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt7;
//     pt7.x = 0.0;
//     pt7.y = 1.0;
//     pt7.z = 0.0;
//     oct.addPoint(&pt7);
//     EXPECT_EQ(7, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt8;
//     pt8.x = 0.0;
//     pt8.y = 0.0;
//     pt8.z = 1.0;
//     oct.addPoint(&pt8);
//     EXPECT_EQ(8, oct.getNumPoints());
//     EXPECT_EQ(1, oct.getDepth());
//     pcl::PointXYZ pt9;
//     pt9.x = 0.0;
//     pt9.y = 0.0;
//     pt9.z = 1.1;
//     oct.addPoint(&pt9);
//     EXPECT_EQ(9, oct.getNumPoints());
//     EXPECT_EQ(2, oct.getDepth());
}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}