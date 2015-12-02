/*
 * Date Started: 02/12/15
 * Original Author: Harry J.E Day <harry@dayfamilyweb.com>
 * Editors:
 * ROS Node Name: camera_fusion_node
 * ROS Package: owr_3d_fusion
 * Purpose: Class used to build an octree
 */



#include "owr_3d_fusion/Octree.h"
#include <malloc.h>
#include <boost/iterator/iterator_concepts.hpp>
#include <string.h>

Octree::Octree() : depthCount(FIRST_GUESS_DEPTH,0) {
    //setup the root node
    head = (TreeNode) malloc(sizeof(treeNode)); 
    head->depth = 0;
    head->isLeaf = true;
    pcl::PointXYZ orig;
    orig.x = 0.0;
    orig.y = 0.0;
    orig.z = 0.0;
    head->origin = orig;
    setChildrenToNull(head);
    
    //setup the depth
    depthCount[0] = 1;
    depth = 1;
    
    //setup the size
    numPoints = 0;
    
}

Octree::~Octree() {
    //TODO: walk the tree and free
    free(head);
}

void Octree::addPoint(pcl::PointXYZ * point) {
    TreeNode t = findLeaf(point);
    addPointRecure(point,t);
}

void Octree::addPointRecure ( pcl::PointXYZ* point, TreeNode t ) {
    const int index = calculateIndex(point,t->origin);
    if(t->isLeaf) {
        //full
        if(t->children.leafChildren[index]) {
            splitLeaf(t);
            addPointRecure(point, t->children.nodeChildren[index]);
        } else {
            t->children.leafChildren[index] = point;
            numPoints++;
        }
        //TODO: account for when this is full
    } else {
        t->children.nodeChildren[calculateIndex(point,t->origin)] = createNewLeaf(t,index);
        addPointRecure(point,t);
    }
}

int Octree::getDepth() {
    return depth;
}

int Octree::getNumPoints() {
    return numPoints;
}

//helper function to split a leaft into 8 leaf nodes and a parent
void Octree::splitLeaf ( TreeNode leaf ) {
    pcl::PointXYZ * leafChildren[NUM_OCT_TREE_CHILDREN];
    memcpy(leafChildren, leaf->children.leafChildren, sizeof(leafChildren));
    leaf->isLeaf = false;
    int i;
    for(i = 0; i < NUM_OCT_TREE_CHILDREN; i++) {
        leaf->children.leafChildren[i] = NULL;
        //TODO: this could be made more efficient by passing the index
        leaf->children.nodeChildren[i] = createNewLeaf(leaf,i);
        addPointRecure(leafChildren[i], leaf->children.nodeChildren[i]);
    }
}


//helper function to set all the children to null
void Octree::setChildrenToNull ( TreeNode node ) {
    int i;
    for(i=0;i<NUM_OCT_TREE_CHILDREN;i++) {
        node->children.nodeChildren[i] = NULL;
    }
}

//helper function to find the leaf releavant to this point
TreeNode Octree::findLeaf ( pcl::PointXYZ * point ) {
    TreeNode current = head;
    TreeNode temp = head;
    while(current->isLeaf != true) {
        temp = head->children.nodeChildren[calculateIndex(point,current->origin)];
        if(temp) {
            current = temp;
        } else {
            break;
        }
    }
    return current;
}

TreeNode Octree::createNewLeaf ( TreeNode parent, int index ) {
    TreeNode n = (TreeNode) malloc(sizeof(treeNode));
    setChildrenToNull(n);
    n->depth = parent->depth+1;
    if(depthCount[n->depth] == 0) {
        depth++;
    }
    n->isLeaf = true;
    
    //increase the depth count
    if(depthCount.size() < n->depth) {
        depthCount.resize(n->depth);
    }
    depthCount[n->depth]++;
    return n;
}



int Octree::calculateIndex ( pcl::PointXYZ * point, pcl::PointXYZ orig ) {
    int oct = 0;
    //8 is 100 in binary
    //position in tree is determined by this algorithm
    if(point->x > orig.x) {
        oct |= 4;
    }
    if(point->y > orig.y) {
        oct |= 2;
    }
    if(point->z > orig.z) {
        oct |= 1;
    }
    return oct;
}



