#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#include </home/ros/owr_software/rover/src/owr_3d_fusion/include/owr_3d_fusion/logitechC920.h>
#include </home/ros/owr_software/rover/src/owr_3d_fusion/include/owr_3d_fusion/OctreeDefines.h>
//see: http://enja.org/2011/03/30/adventures-in-opencl-part-3-constant-memory-structs/
//for OpenCL structs
typedef struct octNode {
    unsigned long locCode;
    uchar childrenMask;
    float3 simplePoint[8];
    float3 orig;
    float3 dimensions;
    bool exists;
} OctNode;

#define CV_Y_INDEX 0
#define CV_X_INDEX 1
#define RES 0.01 //(1cm)
#define TRACE_RANGE 60
#define ROOT_LOC_CODE 1

float3 getPointAt(struct octNode node , uchar index) {
    return node.simplePoint[index];
}

uchar calculateIndex(float3 point, float3 orig) {
    int oct = 0;
    //8 is 1000 in binary
    //position in tree is determined by this algorithm
    if(point.x > orig.x) {
        oct |= 4;
    }
    if(point.y > orig.y) {
        oct |= 2;
    }
    if(point.z > orig.z) {
        oct |= 1;
    }
    return oct;
}

unsigned long getMaskFromIndex(const int index) {
    unsigned long childMask = 0;
    switch(index) {
        case 0:
            childMask = CHILD_1;
            break;
        case 1:
            childMask = CHILD_2;
            break;
        case 2:
            childMask = CHILD_3;
            break;
        case 3:
            childMask = CHILD_4;
            break;
        case 4:
            childMask = CHILD_5;
            break;
        case 5:
            childMask = CHILD_6;
            break;
        case 6:
            childMask = CHILD_7;
            break;
        case 7:
            childMask = CHILD_8;
            break;
        default: 
            childMask = 0;
            break;
    }
    return childMask;
}

struct octNode getNodeAt(constant const struct octNode * tree, const unsigned long locCode) {
    unsigned long hash = (locCode & 0x7FFFFFFF) % HASH_MAP_SIZE;
    struct octNode current;
    do {
        current = tree[hash];
        if(current.locCode != locCode) {
            hash++;
        } else {
            break;
        }
        //TODO: I feel that if the tree gets very full we might need an additional saftey on this
    } while(current.exists);
    return current;
}

struct octNode getNode(float3 pt, constant const struct octNode * tree) {
    unsigned long locCode = ROOT_LOC_CODE;
    struct octNode parent = tree[ROOT_LOC_CODE];
    while(!parent.childrenMask) {
        const int index = calculateIndex(pt, parent.orig);
        const unsigned long childMask = getMaskFromIndex(index);
        if(parent.childrenMask & (childMask)) {
            locCode = locCode << 3;
            locCode |= (unsigned long)index;
            parent = getNodeAt(tree,locCode);
        } else {
            break;
        }
    }
    return parent;
}

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
    //Start of actual code
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
    float8 outcome = INFINITY;
    for(float dist = FOCAL_LENGTH_M; dist < TRACE_RANGE; dist+=RES) {
        
        target = delta*dist;
        target -= (float)FOCAL_LENGTH_M;
        struct octNode node = getNode(target, tree);

        int index = get_global_id(CV_X_INDEX)*dims[0].z + get_global_id(CV_Y_INDEX);
        if(node.dimensions.x <= (float)RES) {
            
            outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );
            break;
        } else if (node.dimensions.x <= RES*8) {
            //if(!node.getPointAt(tree->calculateIndex(target, node.orig)).isEmpty()) {
                    outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );
                    break;
            //}
        } else {
            float3 existingPt = getPointAt(node, calculateIndex(target, node.orig));
            if(existingPt.x == INFINITY) {
                continue;
            }
            existingPt = target-existingPt;
            if(fabs(existingPt.x) <= RES) {
                outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );
            }
        }
        
    }
    int index = get_global_id(CV_X_INDEX)*dims[0].z + get_global_id(CV_Y_INDEX);
    result[index] = outcome;
    
//     int myY = get_global_id(0);
//     int myX = get_global_id(1);
    
    
}
