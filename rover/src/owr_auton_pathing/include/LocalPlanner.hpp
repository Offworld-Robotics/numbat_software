/*
 * LocalPlanenr is the node for reading in (subscribing to) the astar Path
 * and publishing twist velocities to cmdVelToJoints to make the rover
 * naviagate autonomously. This node will also allow for local object detection
 * via the lidar detection system
 *
 * Author: Simon Ireland
 * Date: 5/8/2016
 *
 * NODE NAME: 
 * topics used:
 */
 
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>


#define PATH_TOPIC "owr_auton_pathing"
#define TWIST_TOPIC "owr_auton_twist"
#define LIDAR_TOPIC "NULL"
#define MAX_SPEED 0.7
#define FALSE 0
#define TRUE 1

class LocalPlanner {
    
    public:
        LocalPlanner();
        void run();
        
    protected:
        void pathCallback(const nav_msgs::Path::ConstPtr& pathIn);
        void lidarCallback();
        
    private:
        ros::NodeHandle nh;
        
        
        // For receiving and storing an a astar Path
        ros::Subscriber pathSubscriber;
        nav_msgs::Path navPath;
        bool received;
        
        // For receiving obstacle information from a lidar
        ros::Subscriber lidarSubscriber;
        
        // Publish a Twist msg to cmdVelToJoints
        ros::Publisher twistPublisher;
        
        // Transform listener used to determine current position (using map frame and 'base_link')
        tf::TransformListener tfListener;
        
        // Transform result holder, will hold current position.
        tf::StampedTransform currPosition;
};
