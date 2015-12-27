#include "owr_3d_fusion/Octree.h"
#include <pcl/point_types.h>

#include <gtest/gtest.h>
#include <boost/concept_check.hpp>


TEST(CPURayTraceTest, testConstructor) {
    pcl::PointXYZ pt;
    pt.x = 50.0;
    pt.y = 50.0;
    pt.z = 50.0;
    Octree oct(pt);
    
}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}