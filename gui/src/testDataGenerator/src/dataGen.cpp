#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "../../devel/include/bluesat_owr_protobuf/battery_ros.h"

int main(int argc, char ** argv) {
     ros::init (argc, argv, "dataGen");
    
    ros::NodeHandle node;
    
    //publisher
    ros::Publisher pub = node.advertise<sensor_msgs::NavSatFix>("/gps/fix",  1000);
    ros::Publisher pub2 = node.advertise<bluesat_owr_protobuf::battery_ros>("/status/battery",  1000);
     
    ros::Rate loop_rate(10);
    
    while(ros::ok()) { //loop whilst ros is running   
        //indicate we are sending
        ROS_INFO("Sending\n"); 
        
        sensor_msgs::NavSatFix msg;
        msg.longitude = 151.13921239972115;
        msg.latitude = -33.71817896926869;
        msg.altitude = 0;
        
                
        pub.publish(msg);
        
        bluesat_owr_protobuf::battery_ros msg2;
        msg2.voltage = 1.0f;
        pub2.publish(msg2);
        
        ros::spinOnce();
        msg.longitude = 151.1393103003502;
        msg.latitude = -33.71828605732435;
        msg.altitude = 0;
        
                
        pub.publish(msg);
        
        msg2.voltage = 10.0f;
        pub2.publish(msg2);
        
        loop_rate.sleep();
    }
}
