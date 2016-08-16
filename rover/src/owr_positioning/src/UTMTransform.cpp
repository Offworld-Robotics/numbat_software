/*
 * Date Started: 15/08/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: utm_tf
 * ROS Package: owr_positioning
 * Purpose: Provides a UTM co-ordinate system to our map transform
 */

#include "UTMTransform.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "utm_tf");
    
    UTMTransform p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

UTMTransform::UTMTransform() : tfBuffer(), tfListener(tfBuffer) {
    
    newPosSub = nh.subscribe("/owr/utm_position",1,&UTMTransform::newPositionCallback, this);
    currentTransform.header.frame_id = "world";
    currentTransform.child_frame_id = "map";
    currentTransform.header.seq = 0;
    
}


void UTMTransform::newPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    //lookup the transform from map to rover
    try {
        tf2::Transform roverLocalTransform, newPosMapTransform;
        tf2::fromMsg(msg, roverLocalTransform);
        tfBuffer.transform(roverLocalTransform, newPosMapTransform, "base_link", ros::Time(0), "map");

        geometry_msgs::Transform newTf;
        newTf = tf2::toMsg(newPosMapTransform.inverse());
        currentTransform.transform = newTf;

    } catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void UTMTransform::spin() {
    while(ros::ok()) {
        currentTransform.header.stamp = ros::Time::now();
        currentTransform.header.seq++;
        tfBroadcast.sendTransform(currentTransform);
        ros::spinOnce();
    }
}
