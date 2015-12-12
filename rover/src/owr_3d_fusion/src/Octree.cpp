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
#include <limits>

Octree::Octree() {
    //setup the root node
//     head = (TreeNode) malloc(sizeof(treeNode)); 
//     head->depth = 0;
//     head->isLeaf = true;
//     pcl::PointXYZ orig;
//     orig.x = 0.0;
//     orig.y = 0.0;
//     orig.z = 0.0;
//     head->origin = orig;
//     setChildrenToNull(head);
//     
//     //setup the depth
//     depthCount[0] = 1;
//     depth = 1;
//     
//     //setup the size
//     numPoints = 0;

    //initalise the hashmap
    int i = 0;
    for(i=0;i<HASH_MAP_SIZE;i++) {
        hashMap[i] = NULL;
    }
    
    //create the root
    simplePoint pt = {0.0,0.0,0.0};
    createNewLeaf(0,0,pt);
    
}

Octree::~Octree() {
    //TODO: walk the tree and free
//     free(head);
}

//adds a point from the root
//pre-condition: the root node has been initalized
void Octree::addPoint(pcl::PointXYZ  point) {
    
    addPoint(pclToSimplePoint(point),hashMap[doHash(0)]->data);
}

void Octree::addPoint ( simplePoint pt, octNode parent ) {
    //we have a leaf
    if(!parent.childrenMask) {
        //TODO: leaf code
    } else {
        const int index = calculateIndex(pt,parent.orig);
        //does our index exist
        uint32_t childMask = 0;
        switch(index) {
            case 1:
                childMask = CHILD_1;
                break;
            case 2:
                childMask = CHILD_2;
                break;
            case 3:
                childMask = CHILD_3;
                break;
            case 4:
                childMask = CHILD_4;
                break;
            case 5:
                childMask = CHILD_5;
                break;
            case 6:
                childMask = CHILD_6;
                break;
            case 7:
                childMask = CHILD_7;
                break;
            case 8:
                childMask = CHILD_8;
                break;
            default: 
                childMask = 0;
                break;
        }
        //will be true if the child exists
        if(parent.childrenMask ^ (~childMask)) {
            
        } else {
//            createNewLeaf(parent.locationCode, index, ) 
        }
    }
}

simplePoint Octree::pclToSimplePoint ( pcl::PointXYZ pt ) {
    simplePoint point = {pt.x, pt.y, pt.z};
    return point;
}


// 
// void Octree::addPointRecure ( pcl::PointXYZ* point, TreeNode t ) {
//     const int index = calculateIndex(point,t->origin);
//     if(t->isLeaf) {
//         //full
//         if(t->children.leafChildren[index]) {
//             splitLeaf(t);
//             addPointRecure(point, t->children.nodeChildren[index]);
//         } else {
//             t->children.leafChildren[index] = point;
//             numPoints++;
//         }
//         //TODO: account for when this is full
//     } else {
//         t->children.nodeChildren[calculateIndex(point,t->origin)] = createNewLeaf(t,index);
//         addPointRecure(point,t);
//     }
// }
// 
// int Octree::getDepth() {
//     return depth;
// }
// 
// int Octree::getNumPoints() {
//     return numPoints;
// }
// 
// //helper function to split a leaft into 8 leaf nodes and a parent
// void Octree::splitLeaf ( TreeNode leaf ) {
//     pcl::PointXYZ * leafChildren[NUM_OCT_TREE_CHILDREN];
//     memcpy(leafChildren, leaf->children.leafChildren, sizeof(leafChildren));
//     leaf->isLeaf = false;
//     int i;
//     for(i = 0; i < NUM_OCT_TREE_CHILDREN; i++) {
//         leaf->children.leafChildren[i] = NULL;
//         //TODO: this could be made more efficient by passing the index
//         leaf->children.nodeChildren[i] = createNewLeaf(leaf,i);
//         addPointRecure(leafChildren[i], leaf->children.nodeChildren[i]);
//     }
// }



octNode Octree::createNewLeaf ( uint32_t locCodeParent, int index, simplePoint orig ) {
    uint32_t locCode = locCodeParent << 3; //shift the code of the parent
    locCode |= (uint32_t)index;
    uint32_t hash = doHash(locCode);
    HashNode node = (HashNode) malloc(sizeof(hashNode));
    node->locationCode = locCode;
    node->next = NULL;
    node->data.locationCode = locCode;
    node->data.childrenMask = 0;
    node->data.orig = orig;
    
    //clear the points
    int i = 0;
    const simplePoint empty = {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
    };
    for(i=0;i<NUM_OCT_TREE_CHILDREN;i++) {
        node->data.points[i] = empty;
    }
    if(hashMap[hash]) {
        HashNode parentNode = NULL;
        HashNode nextNode = hashMap[hash];
        while(parentNode) {
            if(nextNode->locationCode < node->locationCode && nextNode) {
                parentNode = nextNode->next;
            } else {
                parentNode = nextNode;
                break;
            }
        }
        if(parentNode) {
            parentNode->next = node;
        } else {
            hashMap[hash] = node;
        }
    } else {
       hashMap[hash] = node;
    }
    return node->data;
}



uint32_t Octree::doHash ( uint32_t locCode ) {
    //TODO: find a better hash function, this is the one that java supposedly uses
    return (locCode & 0x7FFFFFFF) % NUM_OCT_TREE_CHILDREN;
}

int Octree::calculateIndex ( simplePoint point, simplePoint orig ) {
    int oct = 0;
    //8 is 100 in binary
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


