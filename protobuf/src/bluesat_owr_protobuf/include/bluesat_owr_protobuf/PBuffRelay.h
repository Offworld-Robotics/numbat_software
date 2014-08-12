/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
#ifndef PBuffRelay_H
#define PBuffRelay_H

#include "messages1.pb.h"
#include <ros/ros.h>

class PBuffRelay {
    
    public:
        PBuffRelay();
        void spin();
        
    private:
        ros::Subscriber sub;
};


#endif
