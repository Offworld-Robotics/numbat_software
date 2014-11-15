/*
 * Header class that implements a specific message
 * this is an example
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 15/11/14
 */
 
#define MESSAGE_CLASS bluesat_owr_protobuf_proto::message1
#define MESSAGE_CLASS_ROS bluesat_owr_protobuf::message1_ros
#define TOPIC "/owr_protobuf/message1"

#include "bluesat_owr_protobuf/PBuffRelay.h"
#include "message1.pb.h"
#include "bluesat_owr_protobuf/message1_ros.h"
#include "../src/PBuffRelay.cpp"


class Message1Relay : public PBuffRelay<MESSAGE_CLASS_ROS,MESSAGE_CLASS> {
    public:
        Message1Relay (std::string topic);
    protected:
        MESSAGE_CLASS_ROS doPbuffToROS(MESSAGE_CLASS pbuffMsg);
};


