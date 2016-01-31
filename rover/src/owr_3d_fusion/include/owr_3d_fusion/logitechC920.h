/*
 * Date Started: 14/11/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Stores #defines to configure the node for use with the logitech c920
 */
#ifndef LOGITECH_C920_H
#define LOGITECH_C920_H

// #include <math.h>

//focal length in m
#define FOCAL_LENGTH_M 0.00367 //3.67mm source: https://forums.logitech.com/t5/Webcams/Focal-Length-of-Logitech-HD-Pro-Webcam-C920/td-p/1280722

//the dimensions of the sensor in meters
//source: https://forums.logitech.com/t5/Webcams/Logitech-HD-Pro-Webcam-C920-optical-sensor-size/td-p/807013
#define SENSOR_DIAG_M ((1.0f/3.0f)*0.0244f)



//the aspect ratio of the sensor does not match the aspect ratio of the 1080p output
//so for now I'm going to assume the pixel size from the sensor width

#define RESOLUTION_W 1920
#define RESOLUTION_H 1080

// #define PIXEL_TO_M_RATIO (SENSOR_WIDTH_M/RESOLUTION_W)
// #define M_TO_PIXEL_RATIO (RESOLUTION_W/SENSOR_WIDTH_M)
#define PX_TO_M (SENSOR_DIAG_M/sqrt(pow(RESOLUTION_W,2) + pow(RESOLUTION_H,2)))
#define M_TO_PX (sqrt(pow(RESOLUTION_W,2) + pow(RESOLUTION_H,2))/SENSOR_DIAG_M)
#define FOCAL_LENGTH_P (FOCAL_LENGTH_M * M_TO_PX)

#endif
