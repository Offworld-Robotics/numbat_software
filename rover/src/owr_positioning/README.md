# owr_positioning 
package for managing positioning of BLUEsat Rovers

## provides the following:

### launch files
- `laser.launch` launch file for the LIDAR used by the BLUEtongue Rover
- `lidar_sweep.launch` launches a node that sweeps the lidar gimbal on BLUEtongue
- `optical_ekf.launch` launches ROS's inbuilt ekf system 
- `optical_localization.launch` launches the optical flow node and associated cameras. **Requires the optical flow udev rule**
- `slam.tf` produces tf frames required for hector slam
- `cse_graphslam2d.launch` - graph slam with pure optical flow 
- `cse`

### nodes
- `optical_localization` - takes in four camera outputs and produces  

(and some other old stuff)

# Graph SLAM Setup

This uses the implementation of Graph SLAM provided by CSE UNSW at (http://robolab.cse.unsw.edu.au:4443/rescue/crosbot/wikis/package/crosbot_graphslam)
As well as our own optical flow code for the tracking frame.

## frames

| Frame           | Provided by                  | Used By         | Purpose |
|-----------------|------------------------------|-----------------|---------|
| /slam           | Graph SLAM                   | RViz/GUI        | Mapping |
| /base_footprint | Robot State Publisher (URDF) | graph_slam, ekf | Base of the Robot in the URDF model. Used to define relative position of sensors |
| /odom           | EKF                          | graph_slam      | tracking frame used for motion of the rover |
| /icp (optional) | crosbot_ogmbicp              | graph_slam      | Can be used instead of odom add laser tracking, may be better |


## SLAM and path planning

`cse_graphslam2d_ogmbicp.launch` starts the nodes for SLAM (`crosbot_ogmbicp`, `crosbot_graphslam`) and path planning (`crosbot_explore`, `crosbot_navigation`).

### Using the path planner

Set a goal by publishing a `geometry_msgs::Point` (in the world frame) to `/crosbot_navigation/goal`. This will compute an A* path with the provided grid from `crosbot_graphslam`. The path is dynamically updated based on new data.

Enable navigation along the path by setting `/crosbot_navigation/mode` to RESUME (1). Velocity commands will be published on `/cmd_vel/twist` but these need to be converted to a usable output with Harry's code (which is not merged into this branch) before sending it to the rover. 

Stop the navigation (velocity commands) at any time by setting `/crosbot_navigation/mode` to PAUSE (0). Resuming will continue navigating to the current goal unless a new goal is set. 

The voronoi image can be visualised from `/crosbot_explore/astar_voronoi_image`. White regions are walls. Red regions are areas the robot will not enter/drive in. Green areas are safe to navigate. I don't think the crosbot node publishes an image with the computed path (might be wrong though).

#### Parameters to tune for `crosbot_explore`

| Parameter                 |  Description                                                                       |
|---------------------------|------------------------------------------------------------------------------------|
| maxVel                    |  maximum velocity output                                                           |
| maxTurn                   |  maximum angular velocity (should be small since the rover can't turn on the spot) |
| voronoi.restrict          |  region around wall rover can't enter (>= radius of rover)                         |
| voronoi.partial           |  region around wall rover can't enter if cell partially restricted                 |
| voronoi.expand            |  region around wall rover can enter but should try to avoid                        |
| voronoi.orphan            |  distance between disconnected walls that are joined into a single wall            |
| search.proximity_distance |  how close to the goal to be considered 'at the goal'                              |
| planner.rate_astarReplan  |  frequency of recomputing A* path to goal (default 1 Hz)                           |
