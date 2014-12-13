/*
 * Header class that implements a specific message
 * this is an example
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 13/12/14
 */
 
#define MESSAGE_CLASS bluesat_owr_protobuf_proto::signal
#define MESSAGE_CLASS_ROS bluesat_owr_protobuf::signal_ros
#define TOPIC "/status/signal"

#include "bluesat_owr_protobuf/PBuffRelay.h"
#include "signal.pb.h"
#include "bluesat_owr_protobuf/signal_ros.h"

#include "../src/PBuffRelay.cpp"

class SignalRelay : public PBuffRelay<MESSAGE_CLASS_ROS,MESSAGE_CLASS> {
    public:
        SignalRelay (std::string topic);
        
    protected:
        MESSAGE_CLASS_ROS doPbuffToROS(MESSAGE_CLASS pbuffMsg);
        void reciveMsg(const boost::shared_ptr<MESSAGE_CLASS_ROS const> & msg);
};


