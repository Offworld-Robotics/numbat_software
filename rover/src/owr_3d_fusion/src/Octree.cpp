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

// #define DEBUG
#define ROOT_LOC_CODE 1



//helper functions
static inline cl_float3 ptToClFloat(simplePoint rhs) {
        cl_float3 f;
        f.x= rhs.x;
        f.y = rhs.y;
        f.z = rhs.z;
        return f;
}


inline simplePoint calculateOrigin(const simplePoint parentDimensions, const simplePoint parentOrigin, const int index) {
    simplePoint dimensions = parentDimensions;
    dimensions.x *= 0.5f;
    dimensions.y *= 0.5f;
    dimensions.z *= 0.5f;
    simplePoint orig = parentOrigin;
    //calculate the new origin by spliting the existing 
    orig.x += dimensions.x * (index & 4 ? .5f : -.5f);
    orig.y += dimensions.y * (index & 2 ? .5f : -.5f);
    orig.z += dimensions.z * (index & 1 ? .5f : -.5f);
    dimensions.x *= 0.5f;
    dimensions.y *= 0.5f;
    dimensions.z *= 0.5f;
    #ifdef DEBUG
        std::cout << "Calc Orig Point" << orig.x << "," << orig.y << "," << orig.z << "index " << index << std::endl;
    #endif
    return orig;
}

inline uint64_t getMaskFromIndex(const int index) {
    uint64_t childMask = 0;
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

Octree::Octree(pcl::PointXYZ dims) : Octree(pclToSimplePoint(dims)) {
    this->dimensions = pclToSimplePoint(dims);

}

Octree::Octree ( simplePoint dims )  {
    this->dimensions = dims;
    //initalise the hashmap
    int i = 0;
    for(i=0;i<HASH_MAP_SIZE;i++) {
        hashMap[i] = NULL;
    }
    
    //create the root
    simplePoint pt = {0.0,0.0,0.0};
    createNewLeaf(0,ROOT_LOC_CODE,pt,dimensions);
    root = getLoc(ROOT_LOC_CODE);
    
}

Octree::~Octree() {
    //TODO: walk the tree and free
//     free(head);
}

//adds a point from the root
//pre-condition: the root node has been initalized
void Octree::addPoint(pcl::PointXYZ  point) {
    
    addPoint(pclToSimplePoint(point),root->data);
}

void Octree::addPoint ( simplePoint pt, octNode parent ) {
    #ifdef DEBUG
        std::cout << "Adding Point" << pt.x << "," << pt.y << "," << pt.z << "to" << parent.locationCode << std::endl;
    #endif
    const int index = calculateIndex(pt,parent.orig);
    //we have a leaf
    if(!parent.childrenMask) {
        #ifdef DEBUG
            std::cout << "Found a leaf" << std::endl;
        #endif
        //is there already a child at our index?
        if(parent.points[index].x !=  FLOAT_INF) {
//             return;
            //implement spliting
//             uint64_t childMask = 1;
            int i;
            simplePoint dimensions = parent.dimensions;
            dimensions.x *= 0.5f;
            dimensions.y *= 0.5f;
            dimensions.z *= 0.5f;
            if(dimensions.x < MAX_RESOLUTON) {
                //there is already a point close enough that we don't need to 
                //ad a new one
            } else {
                for(i =0; i < NUM_OCT_TREE_CHILDREN; i++) {

                    //does a child exist for this index
                    if(parent.points[i].x !=  FLOAT_INF) {
                        simplePoint orig = calculateOrigin(parent.dimensions, parent.orig, i);
                        
                        addPoint(parent.points[i], createNewLeaf(parent.locationCode, i, orig, dimensions ));
                    }
                }
                //lookup the child and continue
                uint64_t locCode = parent.locationCode << 3; //shift the code of the parent
                locCode |= (uint64_t)index;
                addPoint(pt, getLoc(locCode)->data);
            }
        } else {
            //just store it there
            hashMap[doHash(parent.locationCode)]->data.points[index] = pt;
        }
    } else {
//                 return;
        //does our index exist
        const uint64_t childMask = getMaskFromIndex(index);
        //will be true if the child exists
        if(parent.childrenMask & (childMask)) {
            //lookup the child and continue
            uint64_t locCode = parent.locationCode << 3; //shift the code of the parent
            locCode |= (uint64_t)index;
            addPoint(pt, getLoc(locCode)->data);
        } else {
           simplePoint orig = calculateOrigin(parent.dimensions, parent.orig, index);
           //continue deeper
           addPoint(pt, createNewLeaf(parent.locationCode, index, orig, dimensions ));
            
        }
    }
}

inline simplePoint Octree::pclToSimplePoint ( pcl::PointXYZ pt ) {
    simplePoint point = {pt.x, pt.y, pt.z};
    return point;
}



octNode Octree::createNewLeaf ( uint64_t locCodeParent, int index, simplePoint orig, simplePoint dimensions ) {
    uint64_t locCode = locCodeParent << 3; //shift the code of the parent
    locCode |= (uint64_t)index;
    uint64_t hash = doHash(locCode);
    HashNode node = (HashNode) malloc(sizeof(hashNode));
    node->locationCode = locCode;
    node->next = NULL;
    node->data.locationCode = locCode;
    node->data.childrenMask = 0;
    node->data.orig = orig;
    node->data.dimensions = dimensions;
    #ifdef DEBUG
        std::cout << "Adding Leaf" << orig.x << "," << orig.y << "," << orig.z << "at" << locCodeParent << " id:" <<locCode << std::endl;
    #endif
    //clear the points
    int i = 0;
    const simplePoint empty = {
        FLOAT_INF,
        FLOAT_INF,
        FLOAT_INF
    };
    for(i=0;i<NUM_OCT_TREE_CHILDREN;i++) {
        node->data.points[i] = empty;
    }
    if(hashMap[hash]) {
        #ifdef DEBUG
            std::cout << "Conflict" << std::endl;
        #endif
        
        HashNode nextNode = hashMap[hash];
        #ifdef DEBUG
            std::cout  << nextNode << std::endl;
        #endif
        //TODO: sort this
        node->next = nextNode;
    } 
    hashMap[hash] = node;
    if(locCodeParent) {
        getLoc(locCodeParent)->data.childrenMask |= getMaskFromIndex(index);
    }
    return node->data;
}

HashNode Octree::getLoc ( uint64_t locationCode ) {
    HashNode h = hashMap[doHash(locationCode)];
    if(h) {
        #ifdef DEBUG
                std::cout << locationCode << "cmp" << h->locationCode << std::endl;
        #endif
        while(h->locationCode != locationCode) {
            
            if(h->next) {
                h = h->next;
            } else {
                
                h = NULL;
                break;
            }
        }
    }
    return h;
}

/**
 * Retrives the node closest to the point
 */
octNode Octree::getNode ( pcl::PointXYZ point ) {
    //start with the root and burrow
    return getNode(pclToSimplePoint(point));
    //TODO: handle empty
}

/**
 * Retrives the node closest to the point
 */
octNode Octree::getNode ( simplePoint point ) {
    //start with the root and burrow
    return getNode(root->data,point)->data;
    //TODO: handle empty
}

HashNode Octree::getNode ( octNode parent, simplePoint pt ) {
    //leaf
    if(!parent.childrenMask) {
        return getLoc(parent.locationCode);
    } else {
        const int index = calculateIndex(pt, parent.orig);
        const uint64_t childMask = getMaskFromIndex(index);
        //check there is a child
        if(parent.childrenMask & (childMask)) {
            uint64_t locCode = parent.locationCode << 3; //shift the code of the parent
            locCode |= (uint64_t)index;
            return getNode(getLoc(locCode)->data, pt);
        } else {
            return getLoc(parent.locationCode);
        }
    }
}



uint64_t Octree::doHash ( uint64_t locCode ) {
    //TODO: find a better hash function, this is the one that java supposedly uses
    return (locCode & 0x7FFFFFFF) % HASH_MAP_SIZE;
}

//calculates an index between 0 and 7
int Octree::calculateIndex (  simplePoint const& point, simplePoint const& orig ) {
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
    #ifdef DEBUG
        std::cout << "index cacluated" << oct << std::endl;
    #endif
    return oct;
}


simplePoint Octree::getDimensions() {
    return dimensions;
}

octNodeCL * Octree::getFlatTree() {
    octNodeCL * flatMap = (octNodeCL*) malloc(sizeof(octNodeCL)*8*HASH_MAP_SIZE);
    int i;
    for(i=0;i<HASH_MAP_SIZE;i++) {
        flatMap[i].exists = false;
    }
    int target = 0;
    for(i=0;i<HASH_MAP_SIZE;i++) {
        //TODO: sort this
        if(hashMap[i] != NULL) {
            if(target < i) {
                target = i;
            }
            HashNode current = hashMap[i];
            while(current != NULL) {
                while(flatMap[i].exists) {
                    target+=target%HASH_MAP_SIZE;
                }
                flatMap[i].exists = true;
                flatMap[i].locCode = current->data.locationCode;
                
                int indx;
                for(indx= 0; indx < 8;indx++) {
                    flatMap[i].simplePoint[indx] = ptToClFloat(current->data.points[indx]);
                }
                flatMap[i].orig = ptToClFloat(current->data.orig);
                flatMap[i].dimensions = ptToClFloat(current->data.dimensions);
                flatMap[i].childrenMask = current->data.childrenMask;
                current = current->next;
            }
            
        }
    }
    return flatMap;
}
