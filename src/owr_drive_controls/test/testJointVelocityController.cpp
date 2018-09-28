/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Tests the joint controller interface
 */
#include "JointVelocityController.hpp"

#include <gtest/gtest.h>


const double WHEEL_GEARS_1_100[] = {0.01}; //TODO: put in not stupid values here
const double WHEEL_GEARS_1_1[] = {1}; //TODO: put in not stupid values here
// const std::vector < double > SWERVE_GEARS_VECTOR = {0.1, 0.2};
#define WHEEL_N_GEARS 1

//simple test to test the construction of the controller
TEST(TestJointVelController, testConstruction) {
    
    ros::NodeHandle nh("aaa");
    JointVelocityController contrl1(1000, 2000, 24000, 0.5, WHEEL_GEARS_1_1, WHEEL_N_GEARS,  "/test", nh, "test_joint");
}

//simple test to test full reverse, start/stop speeds
TEST(TestJointVelController, testBasicPWM) {
    
    ros::NodeHandle nh("aaa");
    JointVelocityController contrl1(1000, 2000, 14000, 0.5, WHEEL_GEARS_1_1, WHEEL_N_GEARS, "/test", nh, "test_joint");
    EXPECT_EQ(1500, contrl1.velToPWM(0,0));
    
    int max = contrl1.velToPWM(733.04,0);
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    
    
    int min = contrl1.velToPWM(-733.04,0);
    EXPECT_LE(1000, min);
    EXPECT_GE(1100, min);
    printf("minPWM %d, maxPWM %d\n", min, max);
    
}

//test to look at gear ratios
TEST(TestJointVelController, testGearRatios) {
    
    ros::NodeHandle nh("aaa");
    JointVelocityController contrl1(1000, 2000, 14000, 0.5, WHEEL_GEARS_1_100, WHEEL_N_GEARS, "/test", nh, "test_joint");
    EXPECT_EQ(1500, contrl1.velToPWM(0,0));
    
    int max = contrl1.velToPWM(7.3304,0);
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    
    
    int min = contrl1.velToPWM(-7.3304,0);
    EXPECT_LE(1000, min);
    EXPECT_GE(1100, min);
    printf("minPWM %d, maxPWM %d\n", min, max);
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
