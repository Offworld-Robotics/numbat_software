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

//Important: this number must be prime
#define HASH_MAP_SIZE 100003

//The maximum resolution (min dimensions) that new nodes can be created at
#define MAX_RESOLUTON 0.005 //5mm

const float FLOAT_INF = std::numeric_limits<float>::infinity();

typedef struct _simplePoint {
    float x, y, z;
    bool operator==(const _simplePoint& lhs) const {
        return (x == lhs.x) && (y == lhs.y) &&(z == lhs.z);
    }
    _simplePoint operator-(_simplePoint& lhs) const {
        lhs.x-=x;
        lhs.y-=y;
        lhs.z-=z;
        return (lhs);
    }
    inline bool isEmpty() const {
        if(x == FLOAT_INF) {
            return true;
        } else {
            return false;
        }
    }
} simplePoint;

const simplePoint EMPTY_POINT = {
        FLOAT_INF,
        FLOAT_INF,
        FLOAT_INF
};

class octNode {
    public:
        uint64_t locationCode;
        uint8_t childrenMask; //each bit is a node, 1 means it is present
        simplePoint points[NUM_OCT_TREE_CHILDREN]; //the children of leaves
        simplePoint orig;
        simplePoint dimensions; //the dimensions of this node
        //helper functions to abstract this a bit
        //may not be the most efficient
        inline bool isLeaf() const { return (bool) !childrenMask; }
        inline bool hasChildren() const {
            int i = 0;
            for(i=0;i<NUM_OCT_TREE_CHILDREN; i++) {
                if (points[i].x != FLOAT_INF) {
                    return true;
                }
            }
            return false;
        }
        inline simplePoint getPointAt(int i) const { return points[i]; }
};

typedef struct _hashNode *HashNode;

typedef struct _hashNode {
    octNode data;
    uint64_t locationCode;
//     bool hasData;
    HashNode  next;
} hashNode;


class Octree {
    public:
        Octree(pcl::PointXYZ dimensions);
        Octree(simplePoint dimensions);
        ~Octree();
        
        void addPoint(pcl::PointXYZ  point);
        //helper functions
        int getDepth();
        int getNumPoints();
        octNode getNode(pcl::PointXYZ pt);
        octNode getNode(simplePoint pt);
        simplePoint getDimensions();
        int calculateIndex (  simplePoint const& point, simplePoint const& orig );
    private:

        HashNode hashMap[HASH_MAP_SIZE];
        //this makes it more efficent
        HashNode root;
        
        void addPoint(simplePoint pt, octNode parent);
        octNode createNewLeaf(uint64_t parent, int index, simplePoint orig, simplePoint dimensions);
        void splitLeaf(int leaf);

        uint64_t doHash(uint64_t locCode);
        
        inline simplePoint pclToSimplePoint(pcl::PointXYZ pt);
        simplePoint dimensions;
        
        HashNode  getLoc(uint64_t locationCode);
        HashNode getNode(octNode parent, simplePoint pt);
        
        

};

#endif // OCTREE_H
