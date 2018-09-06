#include "CrosbotNavigation.hpp"
#include <crosbot_explore/SetExplorerMode.h>
#include <crosbot_explore/nodes/astarExplorerNode.hpp>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "crosbot_navigation");
    CrosbotNavigation nav;
    nav.run();
}

CrosbotNavigation::CrosbotNavigation() {
    ros::NodeHandle n("~"); // private handle for parameters
    n.param<std::string>("world_frame", world_frame, "icp_test");
    mode_client = nh.serviceClient<crosbot_explore::SetExplorerMode>("/crosbot_explore/explorer_service");
    mode_sub = nh.subscribe<std_msgs::Int32>(TOPIC_MODE, 1, &CrosbotNavigation::receiveModeMsg, this);
    goal_sub = nh.subscribe<geometry_msgs::Point>(TOPIC_GOAL, 1, &CrosbotNavigation::receiveGoalMsg, this);
    mode = PAUSE;
}

void CrosbotNavigation::run() {
    while(ros::ok()) {
        ros::spin();
    }
}

void CrosbotNavigation::receiveModeMsg(const std_msgs::Int32::ConstPtr & mode_msg) {
    setMode(mode_msg->data);
}


void CrosbotNavigation::setMode(int32_t mode) {
    crosbot_explore::SetExplorerMode srv;
    srv.request.mode = mode;
    srv.request.targetOrientation = false;
    if (mode_client.call(srv)) {
		ROS_INFO("CrosbotNavigation mode changed to %d", mode);
        this->mode = mode;
	} else {
		ROS_ERROR("Failed to set CrosbotNavigation mode");
	}
}

void CrosbotNavigation::receiveGoalMsg(const geometry_msgs::Point::ConstPtr & goal_msg) {
    // make sure we are not moving
    if (mode != PAUSE) {
        setMode(PAUSE);
    }
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_msg->x;
    goal.pose.position.y = goal_msg->y;
    goal.pose.position.z = goal_msg->z;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = world_frame;
    crosbot_explore::SetExplorerModeRequest req;
    crosbot_explore::SetExplorerModeResponse resp;
    req.id = 0;
    req.mode = crosbot_explore::MoveMode::MODE_ASTAR; // set astar explore mode
    req.targetOrientation = false; // don't care about reaching orientation of goal
    // add goal to path
    req.path.header.frame_id = goal.header.frame_id;
    req.path.header.stamp = goal.header.stamp;
    req.path.poses.push_back(goal);
    if (mode_client.call(req, resp)) {
		ROS_INFO("Set astar mode to new goal: (%f, %f, %f)", goal_msg->x, goal_msg->y,
                                                            goal_msg->z);
	} else {
		ROS_ERROR("Failed to set new goal");
	}
}
