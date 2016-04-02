/* A Star search
 * By Nuno Das Neves
 * Date 5/3/2016
 * A* path plan based off LIDAR point-cloud
 */

#include "A-star.h"
#include <boost/concept_check.hpp>


void aStar::findPath() {
  
  // example values
  start.x=0;
  start.y=0;
  start.cost = 0;
  start.isStart = true;
  goal.x=20;
  goal.y=20;
  goal.isGoal = true;
  
  currentPos = start;			// set our current position to the start position
  closedSet.push(currentPos);		// add current position to the closed (explored) set
  
  while(currentPos != goal) {
    
    getNeighbors(currentPos); 		// get neighbors of current position
    closedSet.push(openSet.top());	// push the top member of openSet to closedSet
    openSet.pop();			// 
    currentPos = openSet.top(); 	// set current position to the next in the priority queue
    
    if(currentPos == goal) {		// if we're at the goal
      getPath();
      break;
    } else if(openSet.empty()) {		// or if there is no solution (no more valid points)
      printf("no solution");
      break;
    }
  }
}

void aStar::getNeighbors (point point1) {
  point adjacent;
  for(unsigned int i = point1.x-1; i <= point1.x+1; ++i) {
    for(unsigned int j = point1.y-1; j <= point1.y+1; ++j) {
      // if point1 is in the closedSet
      if(std::find(closedSet.begin(), closedSet.end(), point1) != closedSet.end()) {
	continue;
      } else {
	if (!(i == point1.x && j==point1.y)){ 		//we don't want to push the current point!
	  adjacent.stepCost = aStar::occupancyGrid[i][j];
	  adjacent.x = i;
	  adjacent.y = j;
	  if (adjacent.stepCost < 1){				// make sure its passable before using resources on it
	    getF(adjacent);					// get the F cost etc
	    openSet.push(adjacent);				//
	  } else {
	    // if the point is impassible, push it to the closedSet (we never want to expand it)
	    closedSet.push(adjacent);
	  }
	}
      }
    }
  }
}

void aStar::getF (point &point1) {
  point1.goalDist = getDist(point1, goal);		// get distance to goal (our only heuristic right now)
  point1.cost = currentPos.cost + point1.stepCost; 	// get the total cost up to this point
  point1.weight =  point1.cost + point1.goalDist; 	// f = g + h
  point1.previous = &currentPos;
}

double aStar::getDist (point point1, point point2) {
  double dist = sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2)); // euclidean distance
  return dist;
}

void aStar::getPath() {
  while(currentPos.previous != nullptr){
    finalPath.push_back(currentPos);
    currentPos = currentPos->previous;
  }
}

double aStar::getH (point point1, point point2) {
  
}
