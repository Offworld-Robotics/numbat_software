
#include </home/ros/owr_software/rover/src/owr_3d_fusion/include/owr_3d_fusion/logitechC920.h>
//see: http://enja.org/2011/03/30/adventures-in-opencl-part-3-constant-memory-structs/
//for OpenCL structs
typedef struct octNode {
    uchar childrenMask;
    float3 simplePoint[8];
    float3 orig;
    float3 dimensions;
} OctNode;

#define CV_Y_INDEX 0
#define CV_X_INDEX 1
#define RES 0.01 //(1cm)
#define TRACE_RANGE 60



//I think we need a 2D kernel with each worker doing a trace on one part of the image
//otherwise we can't get if there were earlier colissions on the ray
//NOTE: dims is a float3 because it is never used in an integer context. the last dim should be zero
//unless otherwise stated all vectors are in the *ROS* co-ordinate frame. This includes DIM
void kernel rayTrace(global float8 * result, constant const uchar3 * img, constant const struct octNode * tree, constant const float3 * dims) {
    __local float pxToM;
    __local float3 metricOffset;
    
    //TODO: need a more efficient way to do this, see page 37 of intel guide
    if(!get_local_id(0) && !get_local_id(1)) { //in the first of each work group load constants
        pxToM = SENSOR_DIAG_M/sqrt(pow(dims[0].y,2) + pow(dims[0].z,2)) ;
        metricOffset = dims[0] * (2 / pxToM);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    
    float3 metric = (
        0,
        get_global_id(CV_Y_INDEX),
        get_global_id(CV_Y_INDEX));
    metric = metric*(-pxToM) + metricOffset;
    metric.x = 0;
    //TODO: check if this is px or M, we seem to be different things for y and z
    float3 delta = tanh(metric/(float)FOCAL_LENGTH_M);
    //this means we don't have to assign dist latter
    delta.x = 1;
    float3 target;
    for(float dist = FOCAL_LENGTH_M; dist < TRACE_RANGE; dist+=RES) {
        target = delta*dist;
        target -= (float)FOCAL_LENGTH_M;
        //node = tree->getNode(target);
        //BEGIN TEST code, TODO: replace with above line
        struct octNode node = tree[0];
        if(node.dimensions.x <= (float)RES) {
            int index = get_global_id(CV_X_INDEX)*dims[0].z + get_global_id(CV_Y_INDEX);
            result[index] = (target, img[index], 0, 0);
        }
        //END TEST
    }
    //Start of actual code
    int myY = get_global_id(0);
    int myX = get_global_id(1);
    
    
}