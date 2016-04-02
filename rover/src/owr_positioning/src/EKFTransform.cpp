/*
 * Node for running the GPS data through an EKF
 * Author: Elliott Smith for Bluesat OWR
 * Date: 19/03/2016
*/
#include "EKFTransform.h"


int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "owr_ekf_transform_node");
    EKFTransform transformNode();
    transformNode.spin();
    
   


    return EXIT_SUCCESS;
}


EKFTransform::EKFTransform() : node() {
    //subscriber = node.subscribe(//add parameter);
    publisher = node.advertise<owr_messages::positioning>("gps",1000);
}

//void EKFTransform::callback(const boost::shared_ptr<sensor_msg::NavSatFix> &message) {


    


//main loop
void EKFTransform:spin() {
    sensor_msgs::navSatFix msg;
    msg.longitude = 151.2305412;
	msg.latitude = -33.9197674;
    msg.header.frame_id = "base_link";
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.altitude = 0;
    while(ros::ok()) {
        msg.longitude+=0.000011;
        msg.latitude+=0.000012;
        msg.header.seq+=1;
        msg.header.stamp = ros::Time::now();
        ros::spinOnce();
    }
    
}

