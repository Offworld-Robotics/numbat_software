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
#include <../../home/ros/vbox/src/VBox/Devices/EFI/FirmwareOld/BaseTools/Source/C/VfrCompile/Pccts/antlr/antlr.g>

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
    simplePoint points[NUM_OCT_TREE_CHILDREN];
} octNode;

typedef struct hashNode *HashNode;

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
        
        void addPoint(pcl::PointXYZ * point);
        void addPointRecure(pcl::PointXYZ * point, TreeNode t);
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
//         int calculateIndex(pcl::PointXYZ * point, pcl::PointXYZ orig);
        octNode createNewLeaf(octNode parent, int index);
        void splitLeaf(TreeNode leaf);

        uint32_t hash(uint32_t locCode);

};

#endif // OCTREE_H
