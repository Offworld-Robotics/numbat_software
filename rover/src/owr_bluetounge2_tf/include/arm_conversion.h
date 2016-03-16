#ifndef ARM_CONVERSION_H
#define ARM_CONVERSION_H


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class conversion {
private:
    ros::NodeHandle n;
    ros::Subscriber actuator_sub;
    ros::Publisher js_pub;
    tf::TransformBroadcaster broadcaster;
    std::vector<std::map<std::string, double>> inPositions;
    std::vector<std::map<std::string, double>> outPositions

public:
    conversion() {
        inPositions.resize(2);
        outPositions.resize(42); //random number
        actuator_sub = n.subscribe<sensor_msgs::joint_state()
        js_pub = n.advertise<sensor_msgs::joint_state>();
    }
    void actuatorCallBack(const sensor_msgs::joint_state::ConstPtr& actuators);
    void performTransform();
}

#endif
