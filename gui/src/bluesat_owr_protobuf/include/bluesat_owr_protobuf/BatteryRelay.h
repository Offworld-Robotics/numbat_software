/*
 * Header class that implements a specific message
 * this is an example
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 15/11/14
 */
 
#define MESSAGE_CLASS bluesat_owr_protobuf_proto::battery
#define MESSAGE_CLASS_ROS bluesat_owr_protobuf::battery_ros
#define TOPIC "/owr_protobuf/battery"

#include "bluesat_owr_protobuf/PBuffRelay.h"
#include "battery.pb.h"
#include "bluesat_owr_protobuf/battery_ros.h"

#include "../src/PBuffRelay.cpp"

class BatteryRelay : public PBuffRelay<MESSAGE_CLASS_ROS,MESSAGE_CLASS> {
    public:
        BatteryRelay (std::string topic);
        
    protected:
        MESSAGE_CLASS_ROS doPbuffToROS(MESSAGE_CLASS pbuffMsg);
        void reciveMsg(const boost::shared_ptr<MESSAGE_CLASS_ROS const> & msg);
};


