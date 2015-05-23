/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "PositionController.h" 
#include <iostream>
 
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    PositionController p("blah");
    p.spin();
    
    return EXIT_SUCCESS;   
}

void PositionController::PositionController(std::string topic) {
    
    
}

void PositionController::reciveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
    
}
