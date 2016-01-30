//#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#include </home/bluenuc/owr_software/rover/src/owr_3d_fusion/include/owr_3d_fusion/logitechC920.h>
#include </home/bluenuc/owr_software/rover/src/owr_3d_fusion/include/owr_3d_fusion/OctreeDefines.h>
//see: http://enja.org/2011/03/30/adventures-in-opencl-part-3-constant-memory-structs/
//for OpenCL structs
typedef struct octNode {
    float4 simplePoint[8];
    float4 orig;
    unsigned long locCode;
    float dimensions;
    uchar childrenMask;
    uchar exists;
    long8 padding1;
    int8 pading2;
    uchar padding3;
    uchar padding4;
} OctNode;

#define CV_Y_INDEX 0
#define CV_X_INDEX 1
#define RES 0.01 //(1cm)
#define TRACE_RANGE 60
#define ROOT_LOC_CODE 1

float4 getPointAt(constant const struct octNode * node , uchar index) {
    return node->simplePoint[index];
}

uchar calculateIndex(float4 point, float4 orig) {
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

constant const struct octNode * getNodeAt(constant const struct octNode * tree, const unsigned long locCode) {
    unsigned long hash = (locCode & 0x7FFFFFFF) % HASH_MAP_SIZE;
    constant const struct octNode * current;
    do {
        current = &(tree[hash]);
        if(current->locCode != locCode) {
            hash++;
        } else {
            break;
        }
        //TODO: I feel that if the tree gets very full we might need an additional saftey on this
    } while(current->exists);
    return current;
}

constant const struct octNode * getNode(float4 pt, constant const struct octNode * tree) {
    unsigned long locCode = ROOT_LOC_CODE;
    constant const struct octNode * parent = getNodeAt(tree,locCode);
    while(!parent->childrenMask) {
        const unsigned long index = calculateIndex(pt, parent->orig);
        const unsigned long childMask = getMaskFromIndex(index);
        if(parent->childrenMask & (childMask)) {
            locCode = locCode << 3;
            locCode |= index;
            parent = getNodeAt(tree,locCode);
        } else {
            break;
        }
    }
    return parent;
}

//I think we need a 2D kernel with each worker doing a trace on one part of the image
//otherwise we can't get if there were earlier colissions on the ray
//NOTE: dims is a float4 because it is never used in an integer context. the last dim should be zero
//unless otherwise stated all vectors are in the *ROS* co-ordinate frame. This includes DIM
void kernel rayTrace(global float8 * result, constant const uchar3 * img, constant const struct octNode * tree, constant const float4 * dims) {
    __local float pxToM;
    __local float4 metricOffset;
    
    //TODO: need a more efficient way to do this, see page 37 of intel guide
    //if(!get_local_id(0) && !get_local_id(1)) { //in the first of each work group load constants
        pxToM = SENSOR_DIAG_M/sqrt(pow(dims[0].y,2) + pow(dims[0].z,2)) ;
        metricOffset = (dims[0] / 2) * pxToM;
    //}
    //barrier(CLK_LOCAL_MEM_FENCE);
    //Start of actual code
    float4 metric;
    metric.x = 0;
    metric.y = get_global_id(CV_X_INDEX);
    metric.z = get_global_id(CV_Y_INDEX);
    float4 target;
    float8 outcome;
   
    
    metric = metric*(-pxToM) + metricOffset;
    metric.x = 0;
    //TODO: check if this is px or M, we seem to be different things for y and z
    float4 delta = atan(metric/(float)FOCAL_LENGTH_M);
    //this means we don't have to assign dist latter
    delta.x = 1;
    
    outcome.x = INFINITY;
    outcome.y = INFINITY;
    outcome.z = INFINITY;
    //outcome.s3 = metric.x;
    //outcome.s4 = metric.y;
    //outcome.s5 = metric.z;
    
    int index = get_global_id(CV_X_INDEX)*dims[0].z + get_global_id(CV_Y_INDEX);
    for(float dist = FOCAL_LENGTH_M; dist < TRACE_RANGE; dist+=RES) {
        
        target = delta*dist;
        target -= (float)FOCAL_LENGTH_M;
        constant const struct octNode * node = getNode(target, tree);

        
        if(node->dimensions <= (float)RES) {
            
            /*outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );*/
             outcome.x = target.x;
             outcome.y = target.y;
             outcome.z = target.z;
             // outcome = (tree[1].locCode, tree[1].childrenMask, tree[1].simplePoint[0].x,tree[1].simplePoint[0].y, tree[1].simplePoint[0].z);
             //outcome =2;
            break;
        } else if (node->dimensions <= RES*8) {
            //if(!node.getPointAt(tree->calculateIndex(target, node.orig)).isEmpty()) {
                    /*outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );*/
                      outcome.x = target.x;
                      outcome.y = target.y;
                      outcome.z = target.z;
                      //outcome  =3;

                      
                      //outcome = (0,index,0,1);
                    break;
            //}
        } else {
            float4 existingPt = getPointAt(node, calculateIndex(target, node->orig));
            if(existingPt.x == INFINITY) {
                continue;
            }
            existingPt = target-existingPt;
            if(fabs(existingPt.x) <= RES) {
                /*outcome = (target.x,
                               target.y,
                               target.z,
                               img[index].x,
                               img[index].y,
                               img[index].z,
                               0, 0
                              );*/
                outcome.x = target.x;
                outcome.y = target.y;
                outcome.z = target.z; 
            }
        }
        
    }
    result[index] = outcome;
    
//     int myY = get_global_id(0);
//     int myX = get_global_id(1);
    
    
}
