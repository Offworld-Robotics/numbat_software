/* A Star search
 * By Nuno Das Neves
 * Date 5/3/2016 - 19/3/2016
 * A* path plan based off LIDAR point-cloud
 */

#include <iostream>
#include "A-star.h"

/*
 * Instructions:
 * Create an aStar object
 * Give it an occupancyGrid std::vector<std::vector<double> > -1 = impassable
 * set start.x and start.y
 * set goal.x and goal.y
 * run findPath()
 * finalPath now contains the path hopefully
 */


//------------------------test stuff only here

void aStar::printGrid(){   
    // this is how these loops should be set up for correct indexing (pretty sure) i = x; j = y
    for(unsigned int j = 0; j < occupancyGrid[0].size(); ++j) {
      for(unsigned int i = 0; i < occupancyGrid.size(); ++i) {
        point printPoint(i,j);
        if (start.x == i && start.y == j) {
            std::cout << "S ";      // S for start
        } else if (goal.x == i && goal.y == j) {
            std::cout << "G ";      // G for goal
        } else if (occupancyGrid[i][j] == -1) {
            std::cout << "B ";      // B for block
        } else if(std::find(finalPath.begin(), finalPath.end(), printPoint) != finalPath.end()){
            std::cout << "# ";      // # for final path
        } else {
            std::cout << "- ";      // - for empty (passable) grid square
        }
      }
      std::cout << std::endl;
    }
}

//-------------------------end test stuff

void aStar::findPath() {
    currentPos = start;           // set our current position to the start position
    while(currentPos != goal) {
        closedSet.push_back(currentPos);    // push the current point into closedSet
        getNeighbors(currentPos);       // get neighbors of current position
        currentPos = openSet.top();     // set current position to the next in the priority queue
        openSet.pop();
        if(currentPos == goal) {        // if we're at the goal
            getPath();
            break;
        } else if(openSet.empty()) {        // or if there is no solution (no more valid points)
            break;
        }
    }
}

void aStar::getNeighbors (point point1) {
    point adjacent;
    for(int i = point1.x-1; ((i <= point1.x+1) && (i<=9)); ++i) {
        adjacent.x = i;
        for(int j = point1.y-1;((j <= point1.y+1) && (j<=9)); ++j) {
            adjacent.y = j;
            if((std::find(closedSet.begin(), closedSet.end(), adjacent) != closedSet.end()) || (i<0) || (j<0)) {
                continue; // if point1 is in the closedSet we don't want to expand it; continue
            } else {
                if ((point1.x == i) || (point1.y == j)) {
                    adjacent.stepCost = occupancyGrid[i][j];    // if adjacent, use normal stepCost
                } else {
                    // ---this section checks blocks next to adjacent and point1 to see if they're blocked; if both are then it treats adjacent as impassable
                    int xset = adjacent.x+(point1.x - adjacent.x);
                    int yset = adjacent.y+(point1.y - adjacent.y);
                    if ((xset >= 0 && xset <=9) && (yset >= 0 && yset <=9)) {
                        if ((occupancyGrid[xset][adjacent.y] == -1) && (occupancyGrid[adjacent.x][yset] == -1)) {
                            continue;   //not impassable but blocked by adjacent points
                        }
                    }
                    // ---otherwise just add the diagonal stepCost
                    adjacent.stepCost = occupancyGrid[i][j]*sqrt(2);    // if diagonal, multiply stepCost by sqrt(2) because that's how geometry works
                }
                if (adjacent.stepCost >= 0){                    // make sure its passable before using resources on it
                    getF(adjacent);                             // get the F cost etc
                    openSet.push(adjacent);                     // push it to the openSet for future expansion
                } else {
                    // if the point is impassible, push it to the closedSet (we never want to expand it)
                    closedSet.push_back(adjacent);
                }
            }
        }
    }
}

void aStar::getF (point &point1) {
    point1.goalDist = getDist(point1, goal);      // get distance to goal (our only heuristic right now)
    point1.cost = currentPos.cost + point1.stepCost;  // get the total cost up to this point
    point1.weight =  point1.cost + point1.goalDist;   // f = g + h
    point1.previndex = std::find(closedSet.begin(), closedSet.end(), currentPos) - closedSet.begin(); // returns index of a point matching x,y of currentPos in closedSet
}

double aStar::getDist (point point1, point point2) {
    double dist = sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2)); // euclidean distance
    return dist;
}

void aStar::getPath() {
    while(currentPos.previndex != -1){
        finalPath.push_back(currentPos);
        currentPos = closedSet[currentPos.previndex];
    }
}
