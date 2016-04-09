/*
 */
#ifndef A-STAR_H
#include <stdio.h>
#include <stdlib.h>
#include <cmath>        // pow and stuff
#include <vector>       // vector
#include <queue>        // priority_queue
#include <algorithm>        // find

class point {
  public:
    int x;
    int y;
    bool isStart;       // is the start
    bool isGoal;        // is the goal
    double weight;      // f = g + h
    double goalDist;    // distance from this point to the goal
    double stepCost;    // cost to move to this point from any adjacent point
    double cost;        // total cost to get to this point from the start
    point* previous;    // pointer to previous point NOT USED CURRENTLY
    long previndex;     // index of closedSet which is the previous point (better than using a pointer right now...)

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

class aStar {
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
    // -1 = impassable
    std::vector<std::vector<double> > occupancyGrid; // will change to matrix maybe?
    std::vector<point> closedSet;
    typedef std::priority_queue<point, std::vector<point>, comparePoints> comparePointsQueue;
    comparePointsQueue openSet;
    std::vector<point> finalPath;
    point goal;
    point start;
    point currentPos;
  public:
    double getDist(point point1, point point2); // Euclidean distance between 2 points
    void getF(point &point1);                   // Gets all the important stuff for a given point (cost, weight, path...)
    void getNeighbors(point point1);            // Get the neighbors of a point, push to openSet, get weight etc
    void findPath();                            // Main function
    void getPath();                             // Reconstruct path back to start
    void printGrid();                           // TEsT print whole grid and any other relevant stuff
    aStar(comparePointsQueue _openSet, std::vector<std::vector<double>> _occupancyGrid, std::vector<point> _closedSet, std::vector<point> _finalPath, point _goal, point _start, point _currentPos) {
        occupancyGrid = _occupancyGrid;
        openSet = _openSet;
        closedSet = _closedSet;
        finalPath = _finalPath;
        goal = _goal;
        start = _start;
        currentPos = _currentPos;
    }
    aStar() {

    }

};

#endif
