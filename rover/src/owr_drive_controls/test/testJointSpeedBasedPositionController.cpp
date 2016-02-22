/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Tests the joint controller interface
 */
#include "JointSpeedBasedPositionController.hpp"

#include <gtest/gtest.h>

//simple test to test the construction of the controller
TEST(TestJointVelController, testConstruction) {
    
    ros::NodeHandle nh("aaa");
    double gearRatio[2] = {1,2};
    
    JointSpeedBasedPositionController contrl1(0.5,gearRatio, 2, 1000, 2000, 24000,  "/test", nh, "test_joint");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
