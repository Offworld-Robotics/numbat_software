#ifndef ARM_CONVERSION_H
#define ARM_CONVERSION_H


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define RESOLUTION 0.005
#define LOWER_ACTUATOR_LIMIT 0.252
#define UPPER_ACTUATOR_LIMIT 0.199 
#define UPPER_STROKE_NAME "upper_actuator"
#define LOWER_STROKE_NAME "lower_actuator"


static double angle_list[0][2006] = {};

class conversion {
private:
    ros::NodeHandle n;
    ros::Subscriber actuator_sub;
    ros::Publisher js_pub;

    std::map<std::pair<double, double>, std::pair<double, double> > position_map;
    sensor_msgs::JointState angle_msgs;
    std::vector<std::pair<std::string, double> > inPositions;

public:
    void actuatorCallBack(const sensor_msgs::JointState::ConstPtr& actuators);
    void performTransform();
    void initialiseMap();
    conversion() {
        inPositions.resize(2);
        actuator_sub = n.subscribe<sensor_msgs::JointState>("owr/arm_angle", 1, &conversion::actuatorCallBack, this);
        js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    }
};

#endif
