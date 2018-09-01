#include "cmdVelToJoints.hpp"
#include <math.h>

#include <std_msgs/Float64.h>
#include <gtest/gtest.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "owr_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


// test drive mode sub
TEST(TestCmdVelToJoints, testDriveModeSub) {
    ros::NodeHandle nh("aaa");
    ros::Publisher driveMode = nh.advertise<std_msgs::Int16>(TOPIC_MODE,1,true);
    ros::Publisher vels = nh.advertise<geometry_msgs::Twist>(TOPIC_VEL,1,true);
    CmdVelToJoints cmdVelToJoints;
    ros::Rate rate(1);
    std_msgs::Int16 msg;
    msg.data = CRAB;
    driveMode.publish(msg);
    rate.sleep();
    //EXPECT_EQ(CRAB, cmdVelToJoints.getDriveMode());
    msg.data = 2;
    driveMode.publish(msg);
    rate.sleep();
    //EXPECT_EQ(SWERVE, cmdVelToJoints.getDriveMode());
    msg.data = SWERVE;
    driveMode.publish(msg);
    rate.sleep();
    //EXPECT_EQ(FOUR, cmdVelToJoints.getDriveMode());
    geometry_msgs::Twist msg_vel;
    msg_vel.linear.x = 0.1;
    msg_vel.linear.y = 0.2;
    vels.publish(msg_vel);
    rate.sleep();
    msg_vel.linear.x = 0.2;
    msg_vel.linear.y = 0.001;
    vels.publish(msg_vel);
    rate.sleep();
    msg_vel.linear.x = 0.001;
    msg_vel.linear.y = 0.001;
    vels.publish(msg_vel);
    rate.sleep();
    msg_vel.linear.x = 0.1;
    msg_vel.linear.y = -0.2;
    vels.publish(msg_vel);
}
