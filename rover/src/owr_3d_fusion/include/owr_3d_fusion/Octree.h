/*
 * Date Started: 02/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Class used to build an octree
 */
#ifndef OCTREE_H
#define OCTREE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



#define NUM_OCT_TREE_CHILDREN 8
#define FIRST_GUESS_DEPTH 6
//this stores the tree nodes

#define CHILD_1 0b00000001
#define CHILD_2 0b00000010
#define CHILD_3 0b00000100
#define CHILD_4 0b00001000
#define CHILD_5 0b00010000
#define CHILD_6 0b00100000
#define CHILD_7 0b01000000
#define CHILD_8 0b10000000

#define HASH_MAP_SIZE 100000

typedef struct _simplePoint {
    float x, y, z;
} simplePoint;

typedef struct _octnode {
    uint32_t locationCode;
    uint8_t childrenMask; //each bit is a node, 1 means it is present
    simplePoint points[NUM_OCT_TREE_CHILDREN]; //the children of leaves
    simplePoint orig;
    simplePoint dimensions; //the dimensions of this node
} octNode;

typedef struct _hashNode *HashNode;

typedef struct _hashNode {
    octNode data;
    uint32_t locationCode;
//     bool hasData;
    HashNode  next;
} hashNode;


class Octree {
    public:
        Octree(pcl::PointXYZ dimensions);
        ~Octree();
        
        void addPoint(pcl::PointXYZ  point);
        //helper functions
        int getDepth();
        int getNumPoints();
        octNode getNode(pcl::PointXYZ pt);
    private:

        HashNode hashMap[HASH_MAP_SIZE];
        int calculateIndex(simplePoint  point, simplePoint orig);
        void addPoint(simplePoint pt, octNode parent);
        octNode createNewLeaf(uint32_t parent, int index, simplePoint orig, simplePoint dimensions);
        void splitLeaf(int leaf);

        uint32_t doHash(uint32_t locCode);
        
        simplePoint pclToSimplePoint(pcl::PointXYZ pt);
        simplePoint dimensions;
        
        HashNode  getLoc(uint32_t locationCode);
        HashNode getNode(octNode parent, simplePoint pt);

};

#endif // OCTREE_H
