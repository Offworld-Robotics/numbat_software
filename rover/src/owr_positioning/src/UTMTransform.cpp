/*
 * Date Started: 15/08/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: utm_tf
 * ROS Package: owr_positioning
 * Purpose: Provides a UTM co-ordinate system to our map transform
 */

#include "UTMTransform.hpp"


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "utm_tf");
    
    UTMTransform p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

UTMTransform::UTMTransform() {
    
    newPosSub = nh.subscribe("/owr/utm_position",1,&UTMTransform::newPositionCallback, this);
    currentTransform.header.frame_id = "world";
    currentTransform.child_frame_id = "map";
    currentTransform.header.seq = 0;
    
}


void UTMTransform::newPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {

}

void UTMTransform::spin() {
    while(ros::ok()) {
        currentTransform.header.stamp = ros::Time::now();
        currentTransform.header.seq++;
        tfBroadcast.sendTransform(currentTransform);
        ros::spinOnce();
    }
}
