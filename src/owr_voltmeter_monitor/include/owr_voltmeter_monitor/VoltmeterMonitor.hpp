/*
 * Monitors the voltmeter adc and retransmits it to the gui
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS_NODE:owr_voltmeter_monitor
 */
 
#ifndef VOLTMETER_MONITOR_H
#define VOLTMETER_MONITOR_H

#include <ros/ros.h>
#include "owr_messages/adc.h"

class VoltmeterMonitor {
    
    public:
        VoltmeterMonitor();
        void spin();

    private:
        //used to calculate the heading
        void callback(const owr_messages::adc::ConstPtr& joy);
        
        ros::NodeHandle node;
        ros::Publisher  publisher;
        ros::Subscriber subscriber;
        
        float voltmeterScale = 0.0;
        float voltmeterOffset = 0.0;
        std::string voltmeterFrame;
};

#endif //VOLTMETER_MONITOR_H
