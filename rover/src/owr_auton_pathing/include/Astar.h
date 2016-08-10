/*
 */
#ifndef ASTAR_H
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <assert.h>

#include <cmath>        // pow and stuff
#include <vector>       // vector
#include <algorithm>    // find
//#include <array>

#define SIZE_OF_GRID 5000
#define IMPASS 255

#define MAGIC_OFFSET 101.25
#define MAGIC_FACTOR 20

class point {
    public:
        int x;
        int y;
        bool isStart;       // is the start
        bool isGoal;        // is the goal
        double weight;      // f = g + h
        double goalDist;        // distance from this point to the goal
        double stepCost;        // cost to move to this point from any adjacent point
        double cost;        // total cost to get to this point from the start
        point* previous;        // pointer to previous point NOT USED CURRENTLY
        long previndex;        // index of closedSet which is the previous point (better than using a pointer right now...)

        // constructor function; sets default values
        point(int _x=0, int _y=0, bool _isStart=false, bool _isGoal=false, double _weight=0.0f, double _goalDist=0.0f, 
            double _stepCost=0.0f, double _cost=0.0f, point *_previous=NULL, long _previndex=-1) {
            x = _x;
            y = _y;
            isStart = _isStart;
            isGoal = _isGoal;
            weight = _weight;
            goalDist = _goalDist;
            stepCost = _stepCost;
            cost = _cost;
            previous = _previous;
            previndex = _previndex;
        }
        
        /*point() : point(0,0,false, false, 0.0f, 0.0f, 0.0f, 0.0f, NULL, -1) {
            
        }*/
        //friend bool operator==(point lhs, point rhs);
        //friend bool operator!=(point lhs, point rhs);
    
};
bool operator==(const point& lhs, const point& rhs){
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}
bool operator!=(const point& lhs, const point& rhs){
    return (lhs.x != rhs.x || lhs.y != rhs.y);
}

class Astar {
    class comparePoints {
      public:
            comparePoints(){
        }
        bool operator() (const point& lhs, const point& rhs) const
        {
            return lhs.weight > rhs.weight;
        }
    };
    protected:
        ros::Publisher  pathPublisher;
    private:
        
        ros::NodeHandle node;         // ros::NodeHandle nh;
        message_filters::Subscriber<nav_msgs::OccupancyGrid> mapSubscriber;
        ros::Subscriber goalSubscriber;
        ros::Subscriber goSubscriber;
        
        // transform stuff
        tf::TransformListener tfListener;
        tf::StampedTransform transform;
        tf::MessageFilter<nav_msgs::OccupancyGrid> tfFilter;
        
        // these call doSearch after receiving new information
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData);
        void setGoalCallback(const geometry_msgs::PointStamped::ConstPtr& thePoint);
        void setGoCallback(const std_msgs::Bool::ConstPtr& goOrNo);
        
        // do I go or no?
        void doSearch(void);
        bool go;
        
        //nav_msgs::OccupancyGrid inputGrid;      // our inputGrid which we'll convert and search
        nav_msgs::Path finalPath;               // finalPath for us to publish
        
        comparePoints comp;
        std::vector<std::vector <unsigned char> > occupancyGrid;
        std::vector<point> closedSet;
        std::vector<point> openSet;
        std::vector<point> frontierSet;       // gonna use this to keep track of what goes in and out of openSet (can't see it because its an priorityQueue)
        
        std::vector<point> aStarPath;
        
        point goal;
        point start;
        point currentPos;
        
        bool findPath();                          // main astar loop, returns true if successful
        
        void makeGrid(int8_t data[], nav_msgs::MapMetaData info);
        
        void getPath();                           // Reconstruct path back to start
        void convertPath();
        
        void clearPaths();
        
        void computeNeighbors();              // Computes neighbor of a point; gets their weight, pushes to openSet etc
        std::vector<point> getNeighbors(point point1);    // Gets legal (not impassable, not in closedSet) neighbors of a point, returns them with x,y,stepcost defined
        
        bool canGoDiagonal(point point1, point adjacent); // checks if a diagonal movement is legal by checking passability of adjacent points
        void getF(point &point1);             // Gets all the important stuff for a given point (cost, weight, path...)
        double getDist(point point1, point point2);   // Euclidean distance between 2 points
        
        // ----------- testing stuff -------------
        void createGrid(int sizex, int sizey);     // test - for creating an empty occupancyGrid
        void setGridEndPoints(int sx, int sy, int gx,int gy); // test - set start and end point
        void setGridStepCosts(char *typeOfObstacle);          // test - set obstacles
        void printGrid();                 // test print whole grid and any other relevant stuff
        
        // ---------------------------------------
        
    public:
        Astar(const std::string topic);
        
        void spin();
};

#endif
