/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Tests the joint controller interface
 */
#include "LIDARTiltJointController.hpp"

#include <gtest/gtest.h>



//simple test to test the construction of the controller
TEST(TestLIDARController, testConstruction) {
    
    ros::NodeHandle nh("aaa");
    LIDARTiltJointController contrl1(CONTINUOUS,  "/test", nh, "test_joint");
}

//simple test to test STATIONARY
TEST(TestLIDARController, testStationary) {
    
    ros::NodeHandle nh("aaa");
    LIDARTiltJointController contrl1(STATIONARY, "/test", nh, "test_joint");
    EXPECT_EQ(1330, contrl1.velToPWM());
    jointInfo info = contrl1.extrapolateStatus(ros::Time::now(), ros::Time::now());
    EXPECT_EQ (1330,info.pwm);
    EXPECT_EQ (0.0,info.position);
    EXPECT_EQ (0.0,info.velocity);
    
}

//test to test stepping
TEST(TestLIDARController, testStepping) {
    
    ros::NodeHandle nh("aaa");
    LIDARTiltJointController contrl1(CONTINUOUS, "/test", nh, "test_joint");
    EXPECT_EQ(1340, contrl1.velToPWM());
    jointInfo info = contrl1.extrapolateStatus(ros::Time::now(), ros::Time::now());
    EXPECT_EQ (1340,info.pwm);
    EXPECT_GE (0.0215179,info.position);
    EXPECT_LE (0.021516,info.position);
    EXPECT_EQ(1350, contrl1.velToPWM());
    info = contrl1.extrapolateStatus(ros::Time::now(), ros::Time::now());
    EXPECT_EQ (1350,info.pwm);
    EXPECT_GE (0.0430357,info.position);
    EXPECT_LE (0.0430354,info.position);
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
