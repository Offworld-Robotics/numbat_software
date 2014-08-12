/*
 * Rellay Node for transmitting using the protobuf protocol
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 2/08/2014
 */
 
 #include "bluesat_owr_protobuf/PBuffRelay.h"
 
 
 
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
    
    //get a node handle
    
    ros::NodeHandle node("~");
    
    
    bluesat_owr_protobuf::test1 test_message;
    
    
 }
 
 void PBuffRelay::spin() {
    ros::spin();
 }
