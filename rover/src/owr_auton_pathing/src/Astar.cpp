/* A Star search
 * By Nuno Das Neves
 * Date 5/3/2016 - 9/9/2016
 * A* path plan based off SLAM occupancy grid
 */

#include "Astar.h"

int main (int argc, char *argv[]) {
    
    ros::init(argc, argv, "owr_astar");
    //std::cout << "ros init'd" << std::endl;
    Astar pathFinder("owr_auton_pathing");
    pathFinder.spin();
    return 0;
}

// TODO fix for transform
Astar::Astar(const std::string topic) : node(), tfSubscriber(node, "owr_auton_pathing/erc_map", 1), tfFilter(tfSubscriber, tfListener, "base_link", 1) {
    
    mapSubscriber = node.subscribe<nav_msgs::OccupancyGrid>("owr_auton_pathing/erc_map", 2, &Astar::mapCallback, this);
    pathPublisher = node.advertise<nav_msgs::Path>("astarPath", 2, true);
    goalSubscriber = node.subscribe<geometry_msgs::PointStamped>("owr_auton_pathing/astar_goal", 2, &Astar::setGoalCallback, this);
    
    // TODO fix this: should be false by default; use callback to update
    go = true;
    //goSubscriber = node.subscribe<std_msgs::Bool>("owr_auton_pathing/astarstart", 2, &Astar::setGoCallback, this);
    
    tfFilter.registerCallback(boost::bind(&Astar::tfCallback, this, _1));
}



// gets a transform from SLAM (between map and base_link)
void Astar::tfCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData) {
    // allll the spaghetti. Should refactor all this shit later
    gridResolution = gridData->info.resolution;
    
    tfListener.lookupTransform("owr_auton_pathing/erc_map","base_link", gridData->header.stamp, transform);
    // TODO may need offset/scale factor
    start.x = (int)round(transform.getOrigin().getX()/gridResolution);
    start.y = (int)round(transform.getOrigin().getY()/gridResolution);
    start.isStart = true;
    
    ROS_INFO("base_link at (%f, %f)", transform.getOrigin().getX(), transform.getOrigin().getY());
    ROS_INFO("Rover tf at (%d, %d)", start.x, start.y);
    
    // this gets the map...
    makeGrid((int8_t*)gridData->data.data(), gridData->info);
    
    finalPath.header = gridData->header;
    finalPath.header.stamp = ros::Time::now();
    
    if (go == true) {
        doSearch();
    }
}

// gets a map from SLAM
void Astar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData) {
    
    ROS_INFO ("Got a SLAM map");
    start.isStart = true;
    finalPath.header = gridData->header;
    finalPath.header.stamp = ros::Time::now();
    
    start.x = (int)round(gridData->info.width/2/gridResolution);
    start.y = (int)round(gridData->info.height/2/gridResolution);
    start.isStart = true;
    
    if (go == true) {
        doSearch();
    }
}

void Astar::setGoalCallback(const geometry_msgs::PointStamped::ConstPtr& thePoint) {
    // TODO may need an offset/factor
    goal.isGoal = true;
    goal.x = (int)round(thePoint->point.x/gridResolution);
    goal.y = (int)round(thePoint->point.y/gridResolution);
    ROS_INFO("Astar goal at (%d, %d)", goal.x, goal.y);
    
    if (go == true) {
        doSearch();
    }
    
}

void Astar::setGoCallback(const std_msgs::Bool::ConstPtr& goOrNo) {
    
    go = goOrNo->data;
    if (go == false) {
        ROS_INFO("Astar paused");
    } else {
        ROS_INFO("Astar A*ing");
        doSearch();
    }
}

void Astar::doSearch() {
    
    // if it's a nogo, don't go!
    if (go == true && goal.isGoal && start.isStart) {
        
        ROS_INFO("Attempting to find path...");
        if (!findPath()) { // do aStar algorithm, get the path
            ROS_ERROR("No path found!");
            return;
        }
        //printGrid();        // testing only
        convertPath();      // convert aStarPath to the path output we need
        
        clearPaths();
        ROS_INFO("Publishing a path");
        pathPublisher.publish(finalPath);
    }
}

//ros's main loop thing?
void Astar::spin() {
    //std::cout << "spinning" << std::endl;
    while(ros::ok()) {
        ros::spinOnce();
    }
}

void Astar::makeGrid(int8_t data[], nav_msgs::MapMetaData info) {
    
    if((info.width >= SIZE_OF_GRID) || (info.height >= SIZE_OF_GRID)) {
        ROS_ERROR("nav_msgs::OccupancyGrid is too big!");
        return;
    }
    astarGrid.resize(info.width);
    //set entire grid to impassable
    for(unsigned int i = 0; i < astarGrid.size(); ++i) {
        astarGrid[i].resize(info.height);
        for(unsigned int j = 0; j < astarGrid[i].size(); ++j) {
            astarGrid[i][j] = IMPASS;
        }
    }
    
    int x = 0;
    int y = 0;
    char value;
        
    for (unsigned int i = 0; i < (info.width * info.height); ++i) {
        value = data[i];
        
        if (value < 0) {
            value = 100;    // set unknowns to somewhat passable
        }
        astarGrid[x][y] = value;    // set the value
        
        // radius thingy thing
        if (value > IMPASS_THRESHOLD) {
            astarGrid[x][y] = IMPASS;
            // loop through all points potentially in the circle
            for (int c = -IMPASS_RADIUS+1; c < IMPASS_RADIUS; c ++) {
                for (int r = IMPASS_RADIUS-1; r > -IMPASS_RADIUS; r--) {
                    int cx = x+c;
                    int ry = y+r;
                    // make sure the point is in the circle and not outside the astarGrid
                    if (cx < astarGrid.size() && cx >= 0 && ry < astarGrid[0].size() && ry >=0
                        && sqrt((c*c)+(r*r)) <= IMPASS_RADIUS && astarGrid[cx][ry] < IMPASS_THRESHOLD) {
                        astarGrid[cx][ry] = IMPASS_THRESHOLD;
                    }
                }
            }
        }
        
        x ++;   //always increment x
        // if we're at a multiple of the width, move to the next line
        if (x == info.width) {
            y ++;
            x = 0;  // reset x to the start of the row
        }
    }
    
}

void Astar::setGridEndPoints(int sx, int sy, int gx,int gy) {
    start.isStart = true;
    start.x = sx;
    start.y = sy;

    goal.isGoal = true;
    goal.x = gx;
    goal.y = gy;
}

bool Astar::findPath() {
    currentPos = start;           // set our current position to the start position
    
    printf ("start = (%d,%d)\n",start.x,start.y);
    printf ("goal = (%d,%d)\n",goal.x,goal.y);
    
    while(currentPos != goal) {
        
        closedSet.push_back(currentPos);    // push the current point into closedSet
        computeNeighbors();                 // get neighbors of current position
        
        if(openSet.empty()) {               // or if there is no solution (no more valid points)
            return false;
        }
        
        currentPos = openSet[0];     // set current position to the next in the priority queue
        
        std::pop_heap(openSet.begin(), openSet.end(), comp); openSet.pop_back();
        
        //std::cout << "---------------------------" << std::endl;
        if(currentPos == goal) {        // if we're at the goal
            getPath();
            break;
        }
    }
    return true;
}

void Astar::computeNeighbors () {
    std::vector<point> neighbors = getNeighbors(currentPos);
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
    
    for(int i = point1.x-1; ((i <= point1.x+1) && (i < (int)astarGrid.size())); ++i) {  // need to typecast cos i can be negative
        for(int j = point1.y-1;((j <= point1.y+1) && (j < (int)astarGrid.size())); ++j) {
            
            adjacent.x = i;
            adjacent.y = j;
            
            if(std::find(closedSet.begin(), closedSet.end(), adjacent) == closedSet.end() && (i >= 0) && (j >= 0)) {
                
                //std::cout << "i,j = (" << i << ", " << j << ")" << std::endl;

                if (astarGrid[i][j] < IMPASS) {   // if its passable..
                    
                    if ((point1.x == i) || (point1.y == j)) {  
                        //std::cout << "adding a neighbor; adjacent" << std::endl;
                        adjacent.stepCost = astarGrid[i][j];    // if adjacent, use normal stepCost
                        neighbors.push_back(adjacent);
                        
                    } else if (canGoDiagonal(point1, adjacent) == true) {
                        //std::cout << "adding a neighbor; diag" << std::endl;
                        // otherwise just use the diagonal stepCost
                        adjacent.stepCost = astarGrid[i][j]*sqrt(2);    // if diagonal, multiply stepCost by sqrt(2) because that's how geometry works
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
    if ((xset >= 0 && xset < astarGrid.size()) && (yset >= 0 && yset < astarGrid.size())) {
        // now, are those points both impassable? if so, return false
        if ((astarGrid[xset][point1.y] == IMPASS) && (astarGrid[point1.x][yset] == IMPASS)) {
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
    //std::cout << " " << std::endl;
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
    std::vector<point>::iterator it;
    
    while(currentPos.previndex != -1){
        it = aStarPath.begin();
        
        aStarPath.insert(it, currentPos);
        
        currentPos = closedSet[currentPos.previndex];
    }
    //std::cout << "finalpath xy = (" << currentPos.x << ", " << currentPos.y << ")" << std::endl;
    //std::cout << "  finalpath previndex = " << currentPos.previndex << std::endl;
}

void Astar::convertPath() {
    geometry_msgs::PoseStamped thisPose;
    thisPose.header = finalPath.header;
    finalPath.poses.resize(aStarPath.size());
    for (unsigned int i = 0; i < aStarPath.size(); ++i) {
        thisPose.header.seq = i;
        // TODO may need grid offset/scale factor
        thisPose.pose.position.x = (double)aStarPath[i].x*gridResolution;
        thisPose.pose.position.y = (double)aStarPath[i].y*gridResolution;
        
        finalPath.poses[i] = thisPose;
    }
}

void Astar::clearPaths() {
    closedSet.clear();
    openSet.clear();
    frontierSet.clear();
    aStarPath.clear();
}

/*------------------------------------*/
void Astar::printGrid(){
    std::cout << std::endl;
    // this is how these loops should be set up for correct indexing (pretty sure) i = x; j = y
    /*for(unsigned int j = 0; j < astarGrid[0].size(); ++j) {
      for(unsigned int i = 0; i < astarGrid.size(); ++i) {
        point printPoint(i,j);
        if (start.x == i && start.y == j) {
            std::cout << "S ";      // S for start
        } else if (goal.x == i && goal.y == j) {
            std::cout << "G ";      // G for goal
        } else if (astarGrid[i][j] == IMPASS) {
            std::cout << "B ";      // B for block
        } else if(std::find(aStarPath.begin(), aStarPath.end(), printPoint) != aStarPath.end()){
            std::cout << "# ";      // # for final path
        } else {
            std::cout << "- ";      // - for empty (passable) grid square
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
    */
    // print out the actual path
    std::cout << "aStarPath = [";
    for (unsigned int i = 0; i < aStarPath.size(); ++i) {
        std::cout << "(" << aStarPath[i].x << "," << aStarPath[i].y << ")";
        if (i < aStarPath.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
/*------------------------------------*/
