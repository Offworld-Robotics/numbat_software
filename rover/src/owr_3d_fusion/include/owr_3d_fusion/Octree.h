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
typedef struct _treeNode *TreeNode;
typedef struct _treeNode {
    //an octree can have treenodes or points as children, never both
    union  {
        TreeNode nodeChildren[NUM_OCT_TREE_CHILDREN];
        pcl::PointXYZ * leafChildren[NUM_OCT_TREE_CHILDREN];
    } children;
    //the depth of the node starting from 0
    int depth;
    //if the node is a leaf
    bool isLeaf;
    //the origin of this point
    pcl::PointXYZ origin;
} treeNode;

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
        TreeNode head;
        std::vector<int> depthCount;
        int numPoints;
        int depth;
        
        //helper functions
        void setChildrenToNull(TreeNode node);
        TreeNode findLeaf(pcl::PointXYZ * point);
        int calculateIndex(pcl::PointXYZ * point, pcl::PointXYZ orig);
        TreeNode createNewLeaf(TreeNode parent, int index);
        void splitLeaf(TreeNode leaf);
};

#endif // OCTREE_H
