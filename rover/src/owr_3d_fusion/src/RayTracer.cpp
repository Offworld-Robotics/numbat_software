/*
 * Date Started: 26/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Abstract interface for ray tracing (built this way so we can easily switch to OpenCL)
 */
#include "owr_3d_fusion/RayTracer.hpp"

using namespace boost;

simplePoint DIMS = {50.0, 50.0, 50.0};



void RayTracer::loadImage ( cv::Mat image ) {
    this->image = image;
}

RayTracer::RayTracer() {

}

void RayTracer::setOctree ( Octree * octTree ) {
    tree = octTree;
}

RayTracer::~RayTracer() {

}
