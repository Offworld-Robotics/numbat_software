/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
 #include "bluesat_owr_protobuf/PBuffRelay.h"
 #include <iostream>
 
#define MESSAGE_CLASS bluesat_owr_protobuf_proto::message1
#define MESSAGE_CLASS_ROS bluesat_owr_protobuf::message1_ros
 
 

 
template <class rosMessageType, class pbuffMessageType>
PBuffRelay<rosMessageType,pbuffMessageType>::PBuffRelay
(std::string messageTopic) {
    topic = messageTopic;
    ROS_INFO("Initialising relay");
    node = ros::NodeHandle("~");
}

template <class rosMessageType, class pbuffMessageType>
void PBuffRelay<rosMessageType,pbuffMessageType>::spin() {
    while(ros::ok()) {
        pbuffMessage.ParseFromIstream(&std::cin);
        ros::Publisher pub = node.advertise<rosMessageType>(topic,  1000);        
        ros::spinOnce();
    }
}
 
//Add all used message types here!
//TODO: do this with hash defines
#include "message1.pb.h"
#include "bluesat_owr_protobuf/message1_ros.h"
template class PBuffRelay<bluesat_owr_protobuf::message1_ros, bluesat_owr_protobuf_proto::message1>;
