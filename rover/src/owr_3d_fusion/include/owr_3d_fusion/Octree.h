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

#define CHILD_1 00000001b
#define CHILD_2 00000010b
#define CHILD_3 00000100b
#define CHILD_4 00001000b
#define CHILD_5 00010000b
#define CHILD_6 00100000b
#define CHILD_7 01000000b
#define CHILD_8 10000000b

#define HASH_MAP_SIZE 100000

typedef struct _simplePoint {
    float x, y, z;
} simplePoint;

typedef struct _octnode {
    uint32_t locationCode;
    uint8_t childrenMask; //each bit is a node, 1 means it is present
    simplePoint points[NUM_OCT_TREE_CHILDREN]; //the children of leaves
    simplePoint orig;
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
        Octree();
        ~Octree();
        
        void addPoint(pcl::PointXYZ  point);
        //helper functions
        int getDepth();
        int getNumPoints();
    private:
//         TreeNode head;
//         std::vector<int> depthCount;
//         int numPoints;
//         int depth;
//
        HashNode hashMap[HASH_MAP_SIZE];
        //the root node of the octtree
//         octNode root;
        //helper functions
//         void setChildrenToNull(TreeNode node);
//         TreeNode findLeaf(pcl::PointXYZ * point);
        int calculateIndex(simplePoint  point, simplePoint orig);
        void addPoint(simplePoint pt, octNode parent);
        octNode createNewLeaf(uint32_t parent, int index, simplePoint orig);
        void splitLeaf(int leaf);

        uint32_t doHash(uint32_t locCode);
        
        simplePoint pclToSimplePoint(pcl::PointXYZ pt);

};

#endif // OCTREE_H
