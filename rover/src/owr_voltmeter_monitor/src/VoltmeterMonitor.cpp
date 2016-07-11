/*
 * Monitors the voltmeter adc and retransmits it to the gui
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS_NODE:owr_voltmeter_monitor
 */

#include "owr_voltmeter_monitor/VoltmeterMonitor.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32.h>


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_voltmeter_monitor");
    
    VoltmeterMonitor p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

VoltmeterMonitor::VoltmeterMonitor() : node() {
    
        subscriber = node.subscribe("/owr/adc", 2, &VoltmeterMonitor::callback, this); 
        publisher  = node.advertise<std_msgs::Float32>("/owr/voltmeter", 10);
        
        node.getParam("voltmeter_offset", voltmeterOffset);
        node.getParam("voltmeter_scale", voltmeterScale);
        voltmeterFrame = "pot0"; //default value
        node.getParam("voltmeter_frame", voltmeterFrame); 
        ROS_INFO("Running with scale %f and offset %f", voltmeterScale, voltmeterOffset);
}

void VoltmeterMonitor::callback(const owr_messages::adc_< std::allocator< void > >::ConstPtr& adc) {
    std_msgs::Float32 msg;
    
    for(int i = 0; i < adc->potFrame.size(); ++i) {
        if(adc->potFrame[i].compare(voltmeterFrame) == 0) {
            msg.data = (adc->pot[i] + voltmeterOffset) * voltmeterScale;
            publisher.publish<std_msgs::Float32>(msg);
        }
    }
}



void VoltmeterMonitor::spin() {
    ros::spin();

}


