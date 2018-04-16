/** * Date Started: 25/02/2017
* Original Author: Harry J.E Day
* Editors:
* ROS Node Name: joystick_receiver
* ROS Package: owr_arduino_joystick
* Purpose: Node that pulls data from an arduino and transfers it to the control systems
*/
#ifndef PROJECT_JOYSTICK_RECEIVER_NODE_HPP
#define PROJECT_JOYSTICK_RECEIVER_NODE_HPP

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#define NUM_MSG 8

#define FRONT_LEFT_DRIVE 0
#define FRONT_RIGHT_DRIVE 1
#define BACK_LEFT_DRIVE 2
#define BACK_RIGHT_DRIVE 3
#define FRONT_LEFT_SWERVE 4
#define FRONT_RIGHT_SWERVE 5
#define BACK_LEFT_SWERVE 6
#define BACK_RIGHT_SWERVE 7

 
namespace serial_msg {
    typedef struct message {
        uint32_t startMagic;
        double data[NUM_MSG];
        uint32_t endMagic;
    } __attribute__((packed)) Serial_Msg;
    
    typedef struct in_msg {
        uint32_t startMagic;
    } __attribute__((packed)) In_Msg;
 
    const uint32_t startMagic = 0xFEEDBEEF;
    const uint32_t endMagic = 0xDEADBEEF;
}
 


class Joystick_Receiver_Node {
    public:
        Joystick_Receiver_Node(ros::NodeHandle nh);
        void spin();
        
        void receive_drive_front_left(const std_msgs::Float64::ConstPtr & msg);
        void receive_drive_front_right(const std_msgs::Float64::ConstPtr & msg);
        void receive_drive_back_left(const std_msgs::Float64::ConstPtr & msg);
        void receive_drive_back_right(const std_msgs::Float64::ConstPtr & msg);
        void receive_swerve_front_left(const std_msgs::Float64::ConstPtr & msg);
        void receive_swerve_front_right(const std_msgs::Float64::ConstPtr & msg);
        void receive_swerve_back_left(const std_msgs::Float64::ConstPtr & msg);
        void receive_swerve_back_right(const std_msgs::Float64::ConstPtr & msg);

    private:
        bool open_arduino();
        fd_set uart_set;
        struct timeval timeout;
        int port_fd;
        
        serial_msg::Serial_Msg out_msg;
        
        ros::Subscriber front_left_swerve;
        ros::Subscriber front_left_drive;
        ros::Subscriber front_right_swerve;
        ros::Subscriber front_right_drive;
        ros::Subscriber back_left_swerve;
        ros::Subscriber back_left_drive;
        ros::Subscriber back_right_swerve;
        ros::Subscriber back_right_drive;
        
        bool comm(void *message, int message_len, void *resp, int resp_len);
        
};


#endif //PROJECT_JOYSTICK_RECEIVER_NODE_HPP
