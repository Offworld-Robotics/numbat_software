/*
 * Date Started: 15/08/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: utm_tf
 * ROS Package: owr_positioning
 * Purpose: Provides a GPS co-ordinate system to our map transform
 */

#include "GPSTransform.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "utm_tf");
    
    GPSTransform p;
    p.spin();
    
    return EXIT_SUCCESS;   
}

GPSTransform::GPSTransform() : tfBuffer(), tfListener(tfBuffer) {
    
    newPosSub = nh.subscribe("/gps/fix",1,&GPSTransform::newPositionCallback, this);
    newPosSub = nh.subscribe("/owr/sensors/heading",1,&GPSTransform::newMagCallback, this);
    currentTransform.header.frame_id = "world";
    currentTransform.child_frame_id = "map";
    currentTransform.header.seq = 0;
    
}


void GPSTransform::newPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //lookup the transform from map to rover
    try {
        //tf2::Transform baseLinkToBase = tfBuffer.get

        currentTransform.transform.translation = convertToUTM(*msg);
        currentTransform.header.stamp = msg->header.stamp;

    } catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void GPSTransform::newMagCallback ( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) {
    currentTransform.transform.rotation = msg->pose.pose.orientation;
}


geometry_msgs::Vector3 GPSTransform::convertToUTM(const sensor_msgs::NavSatFix &msg) {
    // this function is based on the algorithm here http://www.movable-type.co.uk/scripts/latlong-utm-mgrs.html
    // which is under the MIT license
    double falseEasting = 500e3;
    double falseNorthing = 10000e4;

    // calculates the longitudinal zone
    double zone = std::floor((msg.longitude + 180) / 6.0);

    // calculate the longitude of the central merridian in radians
    const double lambda0 = ((zone-1)*6 - 180 + 3) * (M_PI/180.0);

    // we don't need to handle Norway or Svalbard. If either of these cases become a thing, refer to above link

    const double latRadians = msg.latitude * (M_PI/180.0);

    const double longRadians = msg.longitude * (M_PI/180.0);

    //WGS84 elipsoid
    const int a = 6378137;
    const double b = 6356752.314245;
    const double f =  1/298.257223563;

    const double utmCentralScale = 0.9996;

    // calc easting and northing

    double e = std::sqrt(f * (2-f)); //eccentricity
    double northing = f / (2 - f); // flattening
    double n2 = northing * northing;
    double n3 = northing * n2;
    double n4 = northing * n3;
    double n5 = northing * n4;
    double n6 = northing * n5;

    double cosLon = std::cos(longRadians);
    double sinLon = std::sin(longRadians);
    double tanLon = std::tan(longRadians);

    double t = std::tan(latRadians); // angle on conformal spehere
    double o = std::sinh(e * atanh(e * t/std::sqrt(1+t*t)));

    double tPrime = t * std::sqrt(1+o*o) - o*sqrt(1 * t * t);

    double squiglePrime = std::atan2(tPrime, cosLon);
    double nPrime = asinh(sinLon / std::sqrt(tPrime * tPrime + cosLon * cosLon));

    double capA = a/(1+northing) * (1 + 1/3*n2 + 1/64*n4 + 1/256*n6);

    double alpha[] = {
            std::numeric_limits<double>::quiet_NaN(),
            1/2*northing - 2/3*n2 + 5/16*n3 +   41/180*n4 -     127/288*n5 +      7891/37800*n6,
            13/48*n2 -  3/5*n3 + 557/1440*n4 +     281/630*n5 - 1983433/1935360*n6,
            61/240*n3 -  103/140*n4 + 15061/26880*n5 +   167603/181440*n6,
            49561/161280*n4 -     179/168*n5 + 6601661/7257600*n6,
            34729/80640*n5 - 3418889/1995840*n6,
            212378941/319334400*n6
    };

    double squigle = squiglePrime;
    double n = nPrime;
    for(int j = 1; j <=6; ++j) {
        squigle += alpha[j] * std::sin(2 * j * squiglePrime) * std::cosh(2 * j * nPrime);
        n +=  alpha[j] * std::cos(2 * j * squiglePrime) * std::sinh(2 * j * nPrime);
    }

    double x = utmCentralScale * capA * n;
    double y = utmCentralScale * capA * squigle;

    // calc convergence

    double pPrime = 1;
    double qPrime = 1;

    for(int j=1; j <= 6; ++j) {
        pPrime += alpha[j] * std::cos(2 * j * squiglePrime) * std::cosh(2 * j * nPrime);
        qPrime += alpha[j] * std::sin(2 * j * squiglePrime) * std::sinh(2 * j * nPrime);
    }

    double yPrime = std::atan(tPrime / std::sqrt(1 + tPrime * tPrime) * tanLon);
    double yPrimePrime = std::atan2(qPrime, pPrime);

    double capY = yPrime + yPrimePrime;

    //scale

    double sinLat = std::sin(latRadians);
    double kPrime = std::sqrt(1 - e*e*sinLat*sinLat) * std::sqrt(1 + t * t) / std::sqrt(tPrime * tPrime * cosLon * cosLon);
    double kPrimePrime = capA / a * std::sqrt(pPrime * pPrime + qPrime * qPrime);

    double k = utmCentralScale * kPrime * kPrimePrime;

    // shift to false origins
    x = x + falseEasting; //make x relative to false easting
    if (y < 0) { // fix for southern hemisphere
        y = y + falseNorthing;
    }

    double convergance = y * (M_PI/180);
    double scale = k;

    geometry_msgs::Vector3 pt;
    pt.x = x;
    pt.y = y;
    pt.z = msg.altitude;

    ROS_INFO("Calc zone %f, x %f, y %f, convergence %f, scale %f", zone, x, y, convergance, scale);

    return  pt;


}

void GPSTransform::spin() {
    //we publish a constant map transform
    geometry_msgs::TransformStamped mapTf;
    mapTf.child_frame_id = "base_footprint";
    mapTf.header.frame_id = "map";
    mapTf.header.seq = 0;
    mapTf.transform.translation.x = 0;
    mapTf.transform.translation.y = 0;
    mapTf.transform.translation.z = 0;

    while(ros::ok()) {
        mapTf.header.stamp = ros::Time::now();
        mapTf.header.seq++;
        tfBroadcast.sendTransform(mapTf);
        //currentTransform.header.stamp = ros::Time::now();
        currentTransform.header.seq++;
        tfBroadcast.sendTransform(currentTransform);
        ros::spinOnce();
    }
}
