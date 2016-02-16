/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Tests the joint controller interface
 */
#include "JointVelocityController.hpp"

#include <gtest/gtest.h>

//simple test to test the construction of the controller
TEST(TestJointVelController, testConstruction) {
    
    ros::NodeHandle nh("aaa");
    JointVelocityController contrl1(1000, 2000, 24000, 0.5, "/test", nh);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
