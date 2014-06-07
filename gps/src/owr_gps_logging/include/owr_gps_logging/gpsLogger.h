/*
 *This service logs the gps output as gpx and kml files
 *Created by Harry J.E Day for BlueSat UNSW
 *Date 30/05/2014
 */
#ifndef GPSLogger_H
#define GPSLogger_H

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#define FLOAT_BYTES 4
#define LOG_FILE "log.kml"



#define KML_FOOTER \
"        </coordinates>" \ 
"      </LineString>" \ 
"    </Placemark>" \ 
"  </Document>" \ 
"</kml>"

class GPSLogger {

    public:
        GPSLogger();
        void spin();
        void reciveMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
        
    private:
        std::string coOrdList;
        ros::Subscriber sub;
};

#endif
