/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include <ros/ros.h>
 #include <sensor_msg/Joy.h>
 
 int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_teleop");
    
    ros::spin();
 }
 
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    /*turtlesim::Velocity vel;
    vel.angular = a_scale_*joy->axes[angular_];
    vel.linear = l_scale_*joy->axes[linear_];
    vel_pub_.publish(vel);*/
}
