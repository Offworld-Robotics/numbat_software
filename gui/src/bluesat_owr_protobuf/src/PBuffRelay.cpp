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
    publisher = node.advertise<rosMessageType>(topic,  1000);
    
}


template <class rosMessageType, class pbuffMessageType>
void PBuffRelay<rosMessageType,pbuffMessageType>::initROStoPBuff() {
    //pass the function that is called when a message is recived
    subscriber = node.subscribe(topic, 1000, &PBuffRelay::reciveMsg, this);
    ros::spin();
    ROS_INFO("spinning");
}
/*
 * Warning: this is an endless loop
 * Logic loop that converts pbuff messages into ros
 */
template <class rosMessageType, class pbuffMessageType>
void PBuffRelay<rosMessageType,pbuffMessageType>::spin() {
    while(ros::ok()) {
        pbuffMessage.ParseFromIstream(&std::cin);
        rosMessageType rosMsg = doPbuffToROS(pbuffMessage);
        publisher.publish(rosMsg);
        ros::spinOnce();
    }
}


//Add all used message types here!
//TODO: do this with hash defines
//#include "message1.pb.h"
//#include "bluesat_owr_protobuf/message1_ros.h"
//template class PBuffRelay<bluesat_owr_protobuf::message1_ros, bluesat_owr_protobuf_proto::message1>;

