/*
 * Date Started: 26/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: A simple CPU based ray tracer used for testing
 */
#ifndef CPU_RAY_TRACER_H
#define CPU_RAY_TRACER_H
#include "RayTracer.hpp"
class CPURayTracer: RayTracer {  
    public:
        void runTraces();
        CPURayTracer() ;
        ~CPURayTracer();
};
#endif