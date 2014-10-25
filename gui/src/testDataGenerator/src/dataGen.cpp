#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char ** argv) {
     ros::init (argc, argv, "dataGen");
    
    ros::NodeHandle node;
    
    //publisher
    ros::Publisher pub = node.advertise<sensor_msgs::NavSatFix>("/gps/fix",  1000);
     
    ros::Rate loop_rate(10);
    
    while(ros::ok()) { //loop whilst ros is running   
        //indicate we are sending
        ROS_INFO("Sending\n"); 
        
        sensor_msgs::NavSatFix msg;
        msg.longitude = 151.13921239972115;
        msg.latitude = -33.71817896926869;
        msg.altitude = 0;
        
                
        pub.publish(msg);
        
        ros::spinOnce();
        msg.longitude = 151.1393103003502;
        msg.latitude = -33.71828605732435;
        msg.altitude = 0;
        
                
        pub.publish(msg);
        loop_rate.sleep();
    }
}
