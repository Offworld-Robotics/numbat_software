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
#include <sensor_msgs/LaserScan.h>
#include <time.h>

#include <laser_filters/median_filter.h>

//Topic names
#define PATH_TOPIC "owr_auton_pathing" // Subcribe to the astar path
#define TWIST_TOPIC "/cmd_vel" // Twist being published to the drive controller
#define LIDAR_TOPIC "scan"

#define TEST_PATH "/testPath"

// Constants used by planner

//Max speed [0,1] of the rover. The autonomous will always drive at this speed
#define MAX_SPEED 0.5

// NOTE: Only edit MAX_ANGLE ***********
#define MAX_ANGLE 10                //in degrees, defines the range in which rover will stop
#define MAX_TURN (MAX_ANGLE / 180.0)    //turning 90deg and instead turn relative to the angle offset

// Define when we are close enough to the target
#define MIN_DISTANCE_TO_GOAL 1.5 // in metres

#define MAX_TURN_RADIUS 2.0 //in metres, take into account rover is X m wide

#define DIFF_TURN_ANGLE 20.0 * (M_PI / 180.0) // How much leeway is given on either side of scan
#define MIN_RANGE_OUTER_TURN 0.5 //metres. Leeway given for rover on outer side of turning circle

#define HALF_ROVER_WIDTH 0.8

#define FALSE 0
#define TRUE 1

class LocalPlanner {
    
    public:
        LocalPlanner();
        void run();
        
    protected:
        void pathCallback(const nav_msgs::Path::ConstPtr& pathIn);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle nh;
        
        // For showing the format of a navPath in topic monitor
        ros::Publisher testPublisher;
        
        // For receiving and storing an a astar Path
        ros::Subscriber pathSubscriber;
        nav_msgs::Path navPath;
        bool receivedPath;
        int count;
        
        
        // Publish a Twist msg to cmdVelToJoints
        ros::Publisher twistPublisher;
        
        // transform stuffs
        message_filters::Subscriber<nav_msgs::OccupancyGrid> mapSubscriber;
        tf::TransformListener tfListener;
        tf::StampedTransform transform;
        tf::MessageFilter<nav_msgs::OccupancyGrid> tfFilter;
        
        // Callback for the map updates
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData);
        
        
        // For receiving obstacle information from a lidar
        ros::Subscriber lidarSubscriber;
        sensor_msgs::LaserScan laser;
        bool receivedLaser;
        
        // Scales the map axes to be in metres
        double scaleMap(double value);
        
        // Transform result holder, will hold current position.
        tf::StampedTransform currPosition;
        
        laser_filters::LaserMedianFilter laserFilter;
        //TODO: laser array filter
        
        
        void lidarAvoidance( void);
        //fLFCollision = FALSE;
        bool fLHCollision;
        bool fRHCollision;
        //fRFCollision = FALSE;
        bool fCollision;
        bool bRCollision;
        bool bLCollision;
};
