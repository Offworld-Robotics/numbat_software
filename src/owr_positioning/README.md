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



