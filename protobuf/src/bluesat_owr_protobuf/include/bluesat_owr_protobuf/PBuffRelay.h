/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef PBuffRelay_H
#define PBuffRelay_H

#include <ros/ros.h>

template <class rosMessageType, class pbuffMessageType> class PBuffRelay {
    
    public:
        PBuffRelay(std::string topic);
        void spin();
    protected:
        virtual rosMessageType doPbuffToROS(pbuffMessageType pbuffMsg) {};
        virtual void reciveMsg(boost::shared_ptr<rosMessageType const> msg) {};
        ros::Publisher  publisher;   
        ros::Subscriber subscriber;  
    
    private:
        ros::Subscriber sub;
        pbuffMessageType pbuffMessage;
        ros::NodeHandle node;
        std::string topic;
   
};


#endif
