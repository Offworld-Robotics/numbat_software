/*
 */
#ifndef A-STAR_H
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <boost/concept_check.hpp>

class aStar {
private:
  std::vector<std::vector<double>> occupancyGrid; //will change to matrix
  std::vector<point> closedSet;
  typedef std::priority_queue<point, std::vector<point>, comparePoints> openSet;
  std::vector<point> finalPath;
  point goal;
  point start;
  point currentPos;
public:
  double getDist(point point1, point point2);	// Euclidean distance between 2 points
  double getH(point point1, point point2);	// Not used yet
  void getF(point &point1);			// Gets all the important stuff for a given point (cost, weight, path...)
  void getNeighbors(point point1);		// Get the neighbors of a point, push to openSet, get weight etc
  void findPath();				// Main function
  void getPath();				// Reconstruct path back to start
  aStar() {
    occupancyGrid.resize(20);
    for(unsigned int i = 0; i < occupancyGrid.size(); ++i) {
      occupancyGrid[i].resize(20);
      for(unsigned int j = 0; j < occupancyGrid[i].size(); ++j) {
	occupancyGrid[i][j]=0; // change to random 0-1 later
      }
    }
  }
  class point {
  public:
    int x;
    int y;
    bool isStart;		// is the start
    bool isGoal;		// is the goal
    double weight;		// f = g + h
    double goalDist;		// distance from this point to the goal
    double stepCost;		// cost to move to this point from any adjacent point
    double cost;		// total cost to get to this point from the start
    point * previous;		// [x,y] of previous point
    
    // I forget what this does
    point(int _x=0, int _y=0, bool _isStart=false, bool _isGoal=false, double _weight=0.0f, double _goalDist=0.0f, 
	  double _stepCost=0.0f, double _cost=0.0f, point * _previous=nullptr) {
      x = _x;
      y = _y;
      isStart = _isStart;
      isGoal = _isGoal;
      weight = _weight;
      goalDist = _goalDist;
      stepCost = _stepCost;
      cost = _cost;
      previous = _previous;
    }
    friend operator==(point lhs, point rhs);
    friend operator!=(point lhs, point rhs);
  };
  class comparePoints {
  public:
    comparePoints();
    bool operator() (const point& lhs, const point& rhs) const
    {
      return lhs.cost < rhs.cost;
    }
  };
};

aStar::point::operator==(point lhs, point rhs){
  return (lhs.x == rhs.x && lhs.y == rhs.y);
}
aStar::point::operator!=(point lhs, point rhs){
  return !aStar::point::operator==(lhs, rhs);
}
#endif