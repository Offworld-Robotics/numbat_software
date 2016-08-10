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
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <nav_msgs/OccupancyGrid.h>


//Topic names
#define PATH_TOPIC "owr_auton_pathing"
#define TWIST_TOPIC "owr_auton_twist"
#define LIDAR_TOPIC "NULL"

// Constants used by planner

#define MAX_SPEED 0.7 //Max speed [0,1] of the rover. The autonomous will always drive at this speed

// NOTE: Only edit MAX_ANGLE ***********
#define MAX_ANGLE 10                //in degrees, defines the range in which rover will stop
#define MAX_TURN MAX_ANGLE / 180    //turning 90deg and instead turn relative to the angle offset

#define MIN_DISTANCE_TO_GOAL 0.8 // in metres

// TODO: If a deadzone is desired, implement these
//#define MIN_ANGLE 0
//#define MIN_TURN MIN_ANGLE/180


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
        int count;
        
        // For receiving obstacle information from a lidar
        ros::Subscriber lidarSubscriber;
        
        // Publish a Twist msg to cmdVelToJoints
        ros::Publisher twistPublisher;
        
        // transform stuffs
        message_filters::Subscriber<nav_msgs::OccupancyGrid> mapSubscriber;
        tf::TransformListener tfListener;
        tf::StampedTransform transform;
        tf::MessageFilter<nav_msgs::OccupancyGrid> tfFilter;
        
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData);
        
        double scaleMap(double value);
        
        // Transform result holder, will hold current position.
        tf::StampedTransform currPosition;
        
};
