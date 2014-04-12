/*

This service logs the calls from the husky's encoders
Created by Harry J E Day for BlueSat UNSW

Date 22/03/2014
*/

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#define ARRAY_SIZE 4

void reciveMsg(const sensor_msgs::JointState::ConstPtr& msg) {
        
    ROS_INFO("recived a message");
    const int arraySize = ARRAY_SIZE;//sizeof(msg->name);
    //TODO: This is realy bad pracise, need someway to check the array size!
    int count = 0;
    
    std::ostringstream streamHeader;
    streamHeader << "name";
    streamHeader << "\t";
    streamHeader << "positon";
    streamHeader << "\t";
    streamHeader << "velocity";
    streamHeader << "\t";
    streamHeader << "effort";
    std::string messageHeader(streamHeader.str());
    ROS_INFO(messageHeader.c_str());
    
    
    while(count < arraySize) {
        std::ostringstream stream;
        stream << msg->name[count];
        stream << "\t";
        stream << msg->position[count];
        stream << "\t";
        stream << msg->velocity[count];
        stream << "\t";
        stream << msg->effort[count];
        std::string message(stream.str());
        ROS_INFO(message.c_str());
        count++;
    }
    ROS_INFO((const char*) msg->name[0].c_str());
}

int main(int argc, char** argv) {

    //initialise the ros package
    ros::init(argc, argv, "encoderLogger");

    //a nodehandler is used to communiate with the rest of ros
    ros::NodeHandle n;

    //pass the void that is called when a message is recived
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, reciveMsg);

    //loop with callbacks
    ros::spin();

    return 0;
}

