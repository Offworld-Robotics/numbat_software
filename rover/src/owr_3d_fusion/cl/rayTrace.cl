
#include "owr_3d_fusion/logitechC920.h"
//see: http://enja.org/2011/03/30/adventures-in-opencl-part-3-constant-memory-structs/
//for OpenCL structs
typedef struct octNode {
    uchar childrenMask;
    float3 simplePoint[8];
    float3 orig;
    float3 dimensions;
} OctNode;


//I think we need a 2D kernel with each worker doing a trace on one part of the image
//otherwise we can't get if there were earlier colissions on the ray
void kernel rayTrace(global const float3 * result, read_only const uchar3 * img, read_only const octNode * tree, read_only const int2 * dims) {
    __local float pxToM = SENSOR_DIAG_M/sqrt(pow(dims[0].x,2) + pow(dims[0].y,2)) ;
    __local float2 metricOffset = dims[0]/2 * pxToM; //NOTE: metricOffset.x is Y, and metricOffset.y is Z
    __local float3 metricZ =global_
}