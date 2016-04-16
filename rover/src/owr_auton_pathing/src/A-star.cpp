/* A Star search
 * By Nuno Das Neves
 * Date 5/3/2016 - 19/3/2016
 * A* path plan based off LIDAR point-cloud
 */

#include <iostream>
#include <boost/concept_check.hpp>
#include "Astar.h"

//------------------------test stuff only here

int main(int argc, char **argv) {
    std::cout << "A* search test" << std::endl;
    
    Astar test;
    
    test.setGridEndPoints(0,0,9,9);
    test.setGridStepCosts("middle");
    
    //test.printGrid();
    
    test.findPath();
    
    test.printGrid();
    
    return 0;
}

void Astar::setGridEndPoints(int sx, int sy, int gx,int gy) {
    start.isStart = true;
    start.x = sx;
    start.y = sy;

    goal.isGoal = true;
    goal.x = gx;
    goal.y = gy;
}

void Astar::setGridStepCosts(char *typeOfObstacle) {
    
    // set initial costs to 1
    for(unsigned int i = 0; i < occupancyGrid.size(); ++i) {
        for(unsigned int j = 0; j < occupancyGrid[i].size(); ++j) {
            occupancyGrid[i][j]=1;
        }
    }
    
    int size = occupancyGrid.size();
    
    // add obstacles based on grid size
    if (typeOfObstacle  == "middle"){   // add a block in the middle
        if ((size % 2) == 0) {
            int start = (size/2) - 1;
            int end = (size/2) + 1;
            for(unsigned int p = start; p < end; ++p) {
                occupancyGrid[p][start] = IMPASS;
                occupancyGrid[p][start+1] = IMPASS;
            }
        } else {
             occupancyGrid[size/2][size/2] = IMPASS;
        }
    } else if (typeOfObstacle == "line") {  // add a line in the middle
        int start = (size/3);
        int end = (start*2) - 1;
        for(unsigned int p = start; p < end; ++p) {
            occupancyGrid[size/2][p] = IMPASS;
        }
    }
    
}

void Astar::printGrid(){
    std::cout << std::endl;
    // this is how these loops should be set up for correct indexing (pretty sure) i = x; j = y
    for(unsigned int j = 0; j < occupancyGrid[0].size(); ++j) {
      for(unsigned int i = 0; i < occupancyGrid.size(); ++i) {
        point printPoint(i,j);
        if (start.x == i && start.y == j) {
            std::cout << "S ";      // S for start
        } else if (goal.x == i && goal.y == j) {
            std::cout << "G ";      // G for goal
        } else if (occupancyGrid[i][j] == IMPASS) {
            std::cout << "B ";      // B for block
        } else if(std::find(finalPath.begin(), finalPath.end(), printPoint) != finalPath.end()){
            std::cout << "# ";      // # for final path
        } /*else if (i == currentPos.x && j == currentPos.y) {
            //std::cout << "C ";      // C for current point (for printing from other functions)
        } */else {
            std::cout << "- ";      // - for empty (passable) grid square
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
}

//-------------------------end test stuff

void Astar::findPath() {
    currentPos = start;           // set our current position to the start position
    //int counter = 0;
    while(currentPos != goal) {
        //counter ++;
        //std::cout << "whilecounter: " << counter << std::endl;
        closedSet.push_back(currentPos);    // push the current point into closedSet
        computeNeighbors();       // get neighbors of current position
        
        if(openSet.empty()) {        // or if there is no solution (no more valid points)
            break;
        }
        
        //std::cout << "openSet.top() xy = (" << openSet.top().x << ", " << openSet.top().y << ")" << std::endl;
        std::cout << "  openSet.size()  = " << openSet.size()  << std::endl;
        //std::cout << "  frontierSet size = " << frontierSet.size() << std::endl;
        //std::cout << "  openSet.top().previndex = " << openSet.top().previndex << std::endl;
        
        currentPos = openSet[0];     // set current position to the next in the priority queue
        
        std::pop_heap(openSet.begin(), openSet.end(), comp); openSet.pop_back();
        
        //std::cout << "---------------------------" << std::endl;
        if(currentPos == goal) {        // if we're at the goal
            getPath();
            break;
        }
    }
}

void Astar::computeNeighbors () {
    std::vector<point> neighbors = getNeighbors(currentPos);
    //std::cout << "no. neighbors = " << neighbors.size() << std::endl;
    point adjacent;
    long index;
    std::vector<point>::iterator it;
    double maybeCost;
    
    if (!(neighbors.empty())) {
        for (int i = 0; i < neighbors.size(); ++i) {
            
            adjacent = neighbors[i];
            maybeCost = currentPos.cost + adjacent.stepCost; // get the total cost up to this point
            it = std::find(openSet.begin(), openSet.end(), adjacent);
            
            if (it != openSet.end()){  // check if adjacent is in frontierSet 
                index = it - openSet.begin(); // should return the index of the point...? i guess..?
                
                if (maybeCost >= openSet[index].cost) {
                    continue;   // if in frontierSet (or openSet); continue
                }
                
                //std::cout << "----updating this point xy = (" << frontierSet[frontIndex].x << ", " << frontierSet[frontIndex].y << ")" << std::endl;
                //std::cout << "----with cost = " << maybeCost << std::endl;
                //std::cout << "----which is this index in frontierSet = " << frontIndex << std::endl;
                //std::cout << "----which has this adjacent xy= " << adjacent.x << ", " << adjacent.y << ")" << std::endl;
                openSet[index].cost = maybeCost;
                getF(openSet[index]);
                
            } else {
                
                //std::cout << "adding this point xy = (" << adjacent.x << ", " << adjacent.y << ")" << std::endl;
                adjacent.cost = maybeCost;
                getF(adjacent);                             // get the F cost etc
                openSet.push_back(adjacent); std::push_heap(openSet.begin(), openSet.end(), comp);        // push it to the openSet for future expansion
                
            }      
        }
    }
}

std::vector<point> Astar::getNeighbors(point point1) {
    
    std::vector<point> neighbors;
    point adjacent;
    
    for(int i = point1.x-1; ((i <= point1.x+1) && (i < (int)occupancyGrid.size())); ++i) {  // need to typecast cos i can be negative
        for(int j = point1.y-1;((j <= point1.y+1) && (j < (int)occupancyGrid.size())); ++j) {
            
            adjacent.x = i;
            adjacent.y = j;
            
            if(std::find(closedSet.begin(), closedSet.end(), adjacent) == closedSet.end() && (i >= 0) && (j >= 0)) {
                
                //std::cout << "i,j = (" << i << ", " << j << ")" << std::endl;

                if (occupancyGrid[i][j] < IMPASS) {   // if its passable..
                    
                    if ((point1.x == i) || (point1.y == j)) {  
                        //std::cout << "adding a neighbor; adjacent" << std::endl;
                        adjacent.stepCost = occupancyGrid[i][j];    // if adjacent, use normal stepCost
                        neighbors.push_back(adjacent);
                        
                    } else if (canGoDiagonal(point1, adjacent) == true) {
                        //std::cout << "adding a neighbor; diag" << std::endl;
                        // otherwise just use the diagonal stepCost
                        adjacent.stepCost = occupancyGrid[i][j]*sqrt(2);    // if diagonal, multiply stepCost by sqrt(2) because that's how geometry works
                        neighbors.push_back(adjacent);
                        
                    }
                } else { // if the point is impassible, push it to the closedSet (we never want to expand it)
                    closedSet.push_back(adjacent);
                }
            }
        }
    }
    return neighbors;
}

bool Astar::canGoDiagonal(point point1, point point2) {
    // ---this section checks blocks next to adjacent and point1 to see if they're blocked; if both are then it treats adjacent as impassable
    int xset = point1.x+(point2.x - point1.x);
    int yset = point1.y+(point2.y - point1.y);
    // are xset and yset within the grid (are they real points)
    if ((xset >= 0 && xset < occupancyGrid.size()) && (yset >= 0 && yset < occupancyGrid.size())) {
        // now, are those points both impassable? if so, return false
        if ((occupancyGrid[xset][point1.y] == IMPASS) && (occupancyGrid[point1.x][yset] == IMPASS)) {
            return false;
        }
    }
    return true;
}

void Astar::getF (point &point1) {
    point1.goalDist = getDist(point1, goal);      // get distance to goal (our only heuristic right now)
    //point1.cost = currentPos.cost + point1.stepCost;  // get the total cost up to this point (now done in getNeighbors)
    point1.weight =  point1.cost + point1.goalDist;   // f = g + h
    point1.previndex = std::find(closedSet.begin(), closedSet.end(), currentPos) - closedSet.begin(); // returns index of a point matching x,y of currentPos in closedSet
    //std::cout << "adjacent xy = (" << point1.x << ", " << point1.y << ")" << std::endl;
    //std::cout << "  currentPos.cost = " << currentPos.cost << std::endl;
    //std::cout << "  cost = " << point1.cost << std::endl;
    //std::cout << "  goalDist = " << point1.goalDist << std::endl;
    //std::cout << "  weight = " << point1.weight << std::endl;
    //std::cout << "  previndex xy = (" << closedSet[point1.previndex].x << ", " << closedSet[point1.previndex].y << ")" << std::endl;
}

double Astar::getDist (point point1, point point2) {
    double dist = sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2)); // euclidean distance
    return dist;
}

void Astar::getPath() {
    while(currentPos.previndex != -1){
        finalPath.push_back(currentPos);
        //std::cout << "finalpath xy = (" << currentPos.x << ", " << currentPos.y << ")" << std::endl;
        //std::cout << "  finalpath previndex = " << currentPos.previndex << std::endl;
        currentPos = closedSet[currentPos.previndex];
    }
    //std::cout << "finalpath xy = (" << currentPos.x << ", " << currentPos.y << ")" << std::endl;
    //std::cout << "  finalpath previndex = " << currentPos.previndex << std::endl;
    //std::cout << "---------------------------" << std::endl;
}

void Astar::testFuncs() {
    assert(1==1);   // just in case.
    //createGrid(3, 3);
    setGridEndPoints(0,0,2,2);
    //setGridStepCosts();
    
    //createGrid(30, 30);
    setGridEndPoints(0,0,29,29);
    
    
    //createGrid(300, 300);
    setGridEndPoints(0,0,299,299);
    
    
    //createGrid(3000, 3000);
    setGridEndPoints(0,0,2999,2999);
}
