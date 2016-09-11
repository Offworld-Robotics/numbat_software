/* ERC Competition map publisher
 * By Nuno Das Neves
 * Date 1/9/2016 - 8/9/2016
 * Opens a geotiff map file and publishes it as an occupancyGrid
 */

#ifndef ERCMAPPUB_H
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <cmath>        // abs
#include <vector>       // vector
#include <algorithm>    // find
//#include <array>
// image processing
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
// geotiff data
//#include <gdal/gdal_priv.h>

#define SIZE_OF_GRID 5000
#define IMPASS 255
#define IMPASS_THRESHOLD 200
#define IMPASS_RADIUS 16
//101.25f

// TODO get from geotiff/UTM transform or arg/thing(for map.tif)
#define GRID_OFFSET 0
#define GRID_FACTOR 20
#define IMG_PATH "/home/ros/owr_software/map.tif"

class ERCMapPub {
    protected:
        ros::Publisher  mapPublisher;
        float abs(double adfGeoTransform);
    private:
        ros::NodeHandle node;         // ros::NodeHandle nh;2
        
        nav_msgs::OccupancyGrid outputGrid;      // our outputGrid to help with testing etc?
        void getGeoData();
        void getMap(); // load map from geotif file
    public:
        ERCMapPub(const std::string topic);
        void spin();
};

#endif
