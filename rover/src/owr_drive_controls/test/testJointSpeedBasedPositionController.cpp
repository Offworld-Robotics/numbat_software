/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Tests the joint controller interface
 */
#include "JointSpeedBasedPositionController.hpp"

#include <gtest/gtest.h>

//simple test to test the construction of the controller
TEST(TestJointSpeedController, testConstruction) {
    
    ros::NodeHandle nh("aaa");
    double gearRatio[2] = {1,2};
    
    JointSpeedBasedPositionController contrl1(0.5,gearRatio, 2, 1000, 2000, 24000,  "/test", nh, "test_joint");
}

TEST(TestJointSpeedController, testFullPower) {
    
    ros::NodeHandle nh("aaa");
    double gearRatio[1] = {1};
    
    JointSpeedBasedPositionController contrl1(1,gearRatio, 1, 1000, 2000, 1,  "/test", nh, "test_joint");
    int max = contrl1.posToPWM(M_PI, 0, 5.0); //rotate the maximum possible distance from 0, should require full power
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    printf("max pwm %d\n", max);
    
    int min = contrl1.posToPWM(M_PI+0.001, 0, 5.0); //rotate the maximum possible distance from 0 in the opposite direction, should require full power
    EXPECT_LE(1000, min);
    EXPECT_GE(1100, min);
    printf("min pwm %d\n", min);
    
    int mid = contrl1.posToPWM(0, 0, 5.0); //should be in the middle
    EXPECT_EQ(1500, mid);
    printf("mid pwm %d\n", mid);
}

TEST(TestJointSpeedController, testIncrementing) {
    
    ros::NodeHandle nh("aaa");
    double gearRatio[1] = {1};
    
    JointSpeedBasedPositionController contrl1(0.5,gearRatio, 1, 1000, 2000, 1,  "/test", nh, "test_joint");
    int max = contrl1.posToPWM(M_PI_2, 0, 5.0); //rotate the maximum possible distance from 0, should require full power
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    printf("max pwm %d\n", max);
    
   //but lets say we only got half way 
    max = contrl1.posToPWM(M_PI_2, M_PI/3, 5.0); //rotate the maximum possible distance from 0, should require full power
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    printf("max pwm %d\n", max);
    
    int lower = contrl1.posToPWM(M_PI, M_PI-0.001, 5.0); //getting closer... slow down
    EXPECT_GE(max,lower );
    printf("lower pwm %d\n", lower);
}

TEST(TestJointSpeedController, testGears) {
    
    ros::NodeHandle nh("aaa");
    double gearRatio[1] = {1.0/455.0};
    
    JointSpeedBasedPositionController contrl1(1,gearRatio, 1, 1000, 2000, 1,  "/test", nh, "test_joint");
    int max = contrl1.posToPWM(M_PI, 0, 5.0); //rotate the maximum possible distance from 0, should require full power
    EXPECT_GE(2000,max );
    EXPECT_LE(1900, max);
    printf("max pwm %d\n", max);
    
    int min = contrl1.posToPWM(M_PI+0.001, 0, 5.0); //rotate the maximum possible distance from 0 in the opposite direction, should require full power
    EXPECT_LE(1000, min);
    EXPECT_GE(1100, min);
    printf("min pwm %d\n", min);
    
    int mid = contrl1.posToPWM(0, 0, 5.0); //should be in the middle
    EXPECT_EQ(1500, mid);
    printf("mid pwm %d\n", mid);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
