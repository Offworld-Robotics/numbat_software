/*
 */
#ifndef ASTAR_H
#include <stdio.h>
#include <stdlib.h>
#include <cmath>        // pow and stuff
#include <vector>       // vector
#include <queue>        // priority_queue
#include <set>          // set
#include <algorithm>        // find
#include <assert.h>
#include <boost/concept_check.hpp>

#define SIZE_OF_GRID 10
#define IMPASS 255

class point {
    public:
        int x;
        int y;
        bool isStart;       // is the start
        bool isGoal;        // is the goal
        double weight;      // f = g + h
        double goalDist;        // distance from this point to the goal
        double stepCost;        // cost to move to this point from any adjacent point
        double cost;        // total cost to get to this point from the start
        point* previous;        // pointer to previous point NOT USED CURRENTLY
        long previndex;        // index of closedSet which is the previous point (better than using a pointer right now...)

        // constructor function; sets default values
        point(int _x=0, int _y=0, bool _isStart=false, bool _isGoal=false, double _weight=0.0f, double _goalDist=0.0f, 
            double _stepCost=0.0f, double _cost=0.0f, point *_previous=nullptr, long _previndex=-1) {
            x = _x;
            y = _y;
            isStart = _isStart;
            isGoal = _isGoal;
            weight = _weight;
            goalDist = _goalDist;
            stepCost = _stepCost;
            cost = _cost;
            previous = _previous;
            previndex = _previndex;
        }
        //friend bool operator==(point lhs, point rhs);
        //friend bool operator!=(point lhs, point rhs);
    
};
bool operator==(const point& lhs, const point& rhs){
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}
bool operator!=(const point& lhs, const point& rhs){
    return (lhs.x != rhs.x || lhs.y != rhs.y);
}

/*
class pointSet {
    private:
        std::vector<point> pointVector;
    public:
        void insert(point point1) {
            unsigned int index = (pointVector.size() / 2);   // gets a middleish element
            unsigned int indexSize = index;                  // we need to keep track of how many times we've halved it
            while (indexSize > 0) {
                indexSize /= 2;
                if (pointVector[index].weight > point1.weight) {   
                    index -= indexSize;  
                } else if (pointVector[index].weight < point1.weight) {
                    index += indexSize;
                } else if (pointVector[index].weight == point1.weight) {
                    break;
                }
            }
            iterator it = std::find(pointVector.begin(), pointVector.end(), pointVector[index]);
            pointVector.insert(it, point1); // no idea if this will work..
        }
        int findIndex(point point1) {
            long index = std::find(pointVector.begin(), pointVector.end(), point1);
            if (index != pointVector.end()){  // check if adjacent is in frontierSet 
                return index - pointVector.begin(); // should return the index of the point...? i guess..?
            }
            return -1;
        }
        void pop () {
            
        }
        
};
*/

class Astar {
    class comparePoints {
      public:
            comparePoints(){
        }
        bool operator() (const point& lhs, const point& rhs) const
        {
            return lhs.weight > rhs.weight;
        }
    };
    private:
        comparePoints comp;
        std::array<std::array <unsigned char, SIZE_OF_GRID>, SIZE_OF_GRID > occupancyGrid; // arrays are faster. 255 = impassable
        std::vector<point> closedSet;
        std::vector<point> openSet;
        std::vector<point> frontierSet;       // gonna use this to keep track of what goes in and out of openSet (can't see it because its an priorityQueue)
        std::vector<point> finalPath;
        point goal;
        point start;
        point currentPos;
    public:
        void findPath();                          // main loop
        void getPath();                           // Reconstruct path back to start
        
        void computeNeighbors();              // Computes neighbor of a point; gets their weight, pushes to openSet etc
        std::vector<point> getNeighbors(point point1);    // Gets legal (not impassable, not in closedSet) neighbors of a point, returns them with x,y,stepcost defined

        bool canGoDiagonal(point point1, point adjacent); // checks if a diagonal movement is legal by checking passability of adjacent points
        void getF(point &point1);             // Gets all the important stuff for a given point (cost, weight, path...)
        double getDist(point point1, point point2);   // Euclidean distance between 2 points

        // ----------- TEsTING-STUFF -------------
        void createGrid(int sizex, int sizey);     // TEsT - for creating an empty occupancyGrid
        void setGridEndPoints(int sx, int sy, int gx,int gy); // TEsT - set start and end point
        void setGridStepCosts(char *typeOfObstacle);          // TEsT - set obstacles
        void printGrid();                 // TEsT print whole grid and any other relevant stuff
        void testFuncs();
        // ---------------------------------------

        Astar(std::vector<point> _openSet, std::vector<point> _frontierSet, std::vector<point> _closedSet, std::vector<point> _finalPath, point _goal, point _start, point _currentPos) {
            openSet = _openSet;
            frontierSet = _frontierSet;
            closedSet = _closedSet;
            finalPath = _finalPath;
            goal = _goal;
            start = _start;
            currentPos = _currentPos;
        }
        Astar() {
            
        }
};

#endif
