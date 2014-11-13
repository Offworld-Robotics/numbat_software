/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
 #include "bluesat_owr_protobuf/PBuffRelay.h"
 #include <iostream>
 
 
 
 int main(int argc, char ** argv) {
    //required to make sure protobuf will work correctly
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    
    //init ros
    ros::init(argc, argv, "PBuffRelay");
    
    PBuffRelay relay;
    
    relay.spin();
    
    return EXIT_SUCCESS;   
 }
 
 
 PBuffRelay::PBuffRelay() {
 
    ROS_INFO("Initialising relay");
    node = ros::NodeHandle("~");
    
    
    
    
 }
 
 void PBuffRelay::spin() {
    while(ros::ok()) {
        testMessage.ParseFromIstream(&std::cin);
        ros::Publisher pub = node.advertise<MESSAGE_CLASS_ROS>(TOPIC,  1000);        
        ros::spinOnce();
    }
 }
