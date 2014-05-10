/*
 * converter.cpp
 *
 * This program converts input from the lidar into plc pointCloud messages
 * It broadcasts on /scanConverted
 *
 * WARNING: This code uses a very basic conversion, a more powerfull one is
 * available. Using transform listners. 
 * 
 * Date: 26/04/2014
 * Code by Harry J.E Day on Behalf of BlueSat OWR
 */
 
 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

//this should be very large
#define CACHE_SIZE 100000

laser_geometry::LaserProjection projector_;
//Publisher for /scanConverted
ros::Publisher cloudPublisher;

void reciveMsg(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char ** argv) {

    ros::init (argc, argv, "converter");
    
    ros::NodeHandle node;
    
    //pass the void that is called when a message is recived
    ros::Subscriber sub = node.subscribe("/base_scan/scan", CACHE_SIZE, reciveMsg);   
    
    //let ros know we also want to send messages
    cloudPublisher = node.advertise<sensor_msgs::PointCloud>("/scanConverted",  CACHE_SIZE);

    //loop with callbacks
    ros::spin();
    

    return 0;
}

void reciveMsg(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //stores the pointCloud msg
    sensor_msgs::PointCloud cloud;
    
    //convert
    projector_.projectLaser(*msg, cloud);
    cloudPublisher.publish(cloud);
    
    ros::spinOnce();   
}
