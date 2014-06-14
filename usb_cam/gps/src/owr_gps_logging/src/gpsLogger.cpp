/*
 * GPSLogger Node
 * Logs GPS input to KML
 * By Harry J.E Day for Bluesat OWR
 * Date: 31/05/2014
 */
#include "owr_gps_logging/gpsLogger.h";
#include <fstream>

int main(int argc, char** argv) { 
    
    //initialise the ros package
    ros::init(argc, argv, "gpsLogger");
    GPSLogger gpsLogger;
    
    //loop with callbacks
    gpsLogger.spin();
    return EXIT_SUCCESS;
}

GPSLogger::GPSLogger() {
    ROS_INFO("initlising GPSLogger");
    //a nodehandler is used to communiate with the rest of ros
    ros::NodeHandle n("~");

    //pass the function that is called when a message is recived
    sub = n.subscribe("/gps/fix", 1000, &GPSLogger::reciveMsg, this);
   
    
}

void GPSLogger::spin() {
  
  ros::spin();

}



void GPSLogger::reciveMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    assert(msg);
    
    ROS_INFO("recived a message");
    const int arraySize = 4 ;
    ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
    
    //add the latest corodinates
    std::ostringstream cordStream;
    //check that coOrd is not null
    cordStream << coOrdList;
    cordStream << msg->longitude;
    cordStream << ",";
    cordStream << msg->latitude;
    cordStream << ",";
    cordStream << msg->altitude;
    cordStream << "\n";
    std::string cordStreamString(cordStream.str());
    coOrdList = cordStreamString;
    
    
    
    //open the file
    std::ofstream kmlFile;
    kmlFile.open(LOG_FILE);
    kmlFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	kmlFile << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
	kmlFile << "  <Document>\n";
	kmlFile << "    <name>Paths</name>\n";
	kmlFile << "    <description>ROS Log</description>\n";
	kmlFile << "    <Style id=\"yellowLineGreenPoly\">\n";
	kmlFile << "      <LineStyle>\n";
	kmlFile << "        <color>7f00ffff</color>\n";
	kmlFile << "        <width>4</width>\n";
	kmlFile << "      </LineStyle>\n";
	kmlFile << "      <PolyStyle>\n";
	kmlFile << "        <color>7f00ff00</color>\n";
	kmlFile << "      </PolyStyle>\n";
	kmlFile << "    </Style>\n";
	kmlFile << "    <Placemark>\n";
	kmlFile << "      <name>Absolute Extruded</name>\n";
	kmlFile << "      <description>Transparent green wall with yellow outlines</description>\n";
	kmlFile << "      <styleUrl>#yellowLineGreenPoly</styleUrl>\n";
	kmlFile << "      <LineString>\n";
	kmlFile << "        <extrude>1</extrude>\n";
	kmlFile << "        <tessellate>1</tessellate>\n";
	kmlFile << "        <altitudeMode>absolute</altitudeMode>\n";
	kmlFile << "        <coordinates>\n";
	kmlFile << coOrdList;
    //TODO: add in co-ords
    kmlFile << KML_FOOTER;
    kmlFile.close();
}


