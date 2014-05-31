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
#define LOG_FILE "~/logs/log.kml"
#define KML_HEADER "<?xml version="1.0" encoding="UTF-8"?>" \ 
"<kml xmlns=\"http://www.opengis.net/kml/2.2\">" \ 
"  <Document>" \ 
"    <name>Paths</name>" \ 
"    <description>ROS Log</description>" \ 
"    <Style id=\"yellowLineGreenPoly\">" \ 
"      <LineStyle>" \ 
"        <color>7f00ffff</color>" \ 
"        <width>4</width>" \ 
"      </LineStyle>" \ 
"      <PolyStyle>" \ 
"        <color>7f00ff00</color>" \ 
"      </PolyStyle>" \ 
"    </Style>" \ 
"    <Placemark>" \ 
"      <name>Absolute Extruded</name>" \ 
"      <description>Transparent green wall with yellow outlines</description>" \ 
"      <styleUrl>#yellowLineGreenPoly</styleUrl>" \ 
"      <LineString>" \ 
"        <extrude>1</extrude>" \ 
"        <tessellate>1</tessellate>" \ 
"        <altitudeMode>absolute</altitudeMode>" \ 
"        <coordinates>"


#define KML_FOOTER \
"        </coordinates>" \ 
"      </LineString>" \ 
"    </Placemark>" \ 
"  </Document>" \ 
"</kml>"

class GPSLogger {

    public:
        GPSLogger();
        void reciveMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
    private:
        char * coOrdList;
};

#endif
