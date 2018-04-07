/**
* Date Started: 25/02/2017
* Original Author: Harry J.E Day
* Editors:
* ROS Node Name: joystick_receiver
* ROS Package: owr_arduino_joystick
* Purpose: Node that pulls data from an arduino and transfers it to the control systems
* Based on Bluetongue.cpp
* Released under the MIT License. Copyright BLUEsat UNSW 2017
*/

#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "owr_arduino_joystick/joystick_receiver_node.hpp"
#include "owr_arduino_joystick/communication_structs.hpp"

#define DODGY_USB_CONNECTION 100

 
int main(int argc, char ** argv) {
    //init ros
    ros::init(argc, argv, "owr_joysticks_arduino");
    ros::NodeHandle nh;
    Joystick_Receiver_Node p(nh);
    p.spin();

    return EXIT_SUCCESS;
}

Joystick_Receiver_Node::Joystick_Receiver_Node(ros::NodeHandle nh) {
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    front_left_drive = nh.subscribe<std_msgs::Float64>("/front_left_wheel_axel_controller/command",1, &Joystick_Receiver_Node::receive_drive_front_left, this, transportHints);
    back_left_drive = nh.subscribe<std_msgs::Float64>("/back_left_wheel_axel_controller/command",1, &Joystick_Receiver_Node::receive_drive_back_left, this, transportHints);
    front_right_drive = nh.subscribe<std_msgs::Float64>("/front_right_wheel_axel_controller/command",1, &Joystick_Receiver_Node::receive_drive_front_right, this, transportHints);
    back_right_drive = nh.subscribe<std_msgs::Float64>("/back_right_wheel_axel_controller/command",1, &Joystick_Receiver_Node::receive_drive_back_right, this, transportHints);
    front_left_swerve = nh.subscribe<std_msgs::Float64>("/front_left_swerve_controller/command",1, &Joystick_Receiver_Node::receive_swerve_front_left, this, transportHints);
    back_left_swerve = nh.subscribe<std_msgs::Float64>("/back_left_swerve_controller/command",1, &Joystick_Receiver_Node::receive_swerve_back_left, this, transportHints);
    front_right_swerve = nh.subscribe<std_msgs::Float64>("/front_right_swerve_controller/command",1, &Joystick_Receiver_Node::receive_swerve_front_right, this, transportHints);
    back_right_swerve = nh.subscribe<std_msgs::Float64>("/back_right_swerve_controller/command", 1, &Joystick_Receiver_Node::receive_swerve_back_right, this, transportHints);

}


void Joystick_Receiver_Node::spin() {
    bool ok;
    serial_msg::in_msg from_board;
    while(ros::ok()) {
        while(!open_arduino()) {}
        ROS_INFO("Open Succesfully");
        while(ros::ok()) {
            ok = comm(&out_msg, sizeof(out_msg), &from_board, sizeof(serial_msg::in_msg));
            if(ok) {
                ROS_INFO("Packet Received");
            } else {
                ROS_ERROR("Connection failed");
                break;
            }
            ros::spinOnce();
        }
    }
}


bool Joystick_Receiver_Node::open_arduino() {
    // TODO: write something to find arduinos
    port_fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(port_fd == -1) {
        ROS_ERROR("Error in open uart port");
        return false;
    } else {
        ROS_DEBUG("Open uart port");
    }
    // Set up stuff for select so we can timeout on reads
    FD_ZERO(&uart_set); /* clear the set */
    FD_SET(port_fd, &uart_set); /* add our file descriptor to the set */
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    // Set parameters
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Error Handling
    if (tcgetattr(port_fd, &tty) != 0) {
        ROS_ERROR("Error %d from tcgetattr: %s", errno, strerror(errno));
        close(port_fd);
        return false;
    }
    ROS_DEBUG("Setting up uart");

    // Set Baud Rate
    cfsetospeed(&tty, (speed_t)B19200);
    cfsetispeed(&tty, (speed_t)B19200);

    // Setting other Port Stuff
    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS; // no flow control
    tty.c_cc[VMIN] = 1; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    ROS_DEBUG("About to make raw");

    // Make raw
    cfmakeraw(&tty);

    // Flush Port, then applies attributes
    tcflush(port_fd, TCIFLUSH);
    if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error %d from tcsetattr: %s" ,errno, strerror(errno));
        close(port_fd);
        return false;
    }

    return true;
}

bool Joystick_Receiver_Node::comm(void *message, int message_len, void *resp, int resp_len) {
    ROS_DEBUG("Writing message: ");
    for (int i = 0; i < message_len; i++) {
        ROS_DEBUG("%d: %02x\n", i, *((char *) message + i));
    }
    int written = 0;
    timeout.tv_sec = 0;
    timeout.tv_usec = 400000;
    int empty_writes = 0;
    do {
        int write_amount = write(port_fd, (int8_t*)message + written, message_len - written);
        if (write_amount == -1) {
            ROS_ERROR("USB write error");
            return false;
        }
        written += write_amount;
        if (write_amount == 0) ++empty_writes;
        if (empty_writes > DODGY_USB_CONNECTION) {
            ROS_ERROR("Dodgy usb connection detected");
            return false;
        }
    } while (written < message_len);

    ROS_DEBUG("Written packet, expecting to read %d", resp_len);
    tcflush(port_fd, TCIOFLUSH);
    int empty_reads = 0;
    int readCount = 0;
    bool first_read = false;
    do {
        int rv = select(port_fd + 1, &uart_set, NULL, NULL, &timeout);
        if(rv == -1) {
            ROS_ERROR("select"); /* an error accured */
            return false;
        } else if(rv == 0) {
            ROS_ERROR("timeout"); /* a timeout occured */
            return false; // This might not require a full reset...
        } else {
            int read_amount = read(port_fd, (int8_t*)resp + readCount, resp_len - readCount);
            readCount += read_amount;
            if (read_amount == 0) ++empty_reads;
            if(!first_read) {
                if (read_amount > 1 && *(uint16_t *) resp == 0xBEEF) {
                    first_read = true;
                } else {
                    readCount = 0;
                    continue;
                }
            }
            if (empty_reads > DODGY_USB_CONNECTION) {
                ROS_ERROR("Dodgy usb connection detected");
                return false;
            }
        }
        ROS_DEBUG("reading... %d", readCount);
    } while (readCount < resp_len);
    ROS_DEBUG("Read packet");
    return true;
}

void Joystick_Receiver_Node::receive_drive_front_left(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[FRONT_LEFT_DRIVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_drive_front_right(const std_msgs::Float64::ConstPtr & msg) {
    out_msg.data[FRONT_RIGHT_DRIVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_drive_back_left(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[BACK_LEFT_DRIVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_drive_back_right(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[BACK_RIGHT_DRIVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_swerve_front_left(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[FRONT_LEFT_SWERVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_swerve_front_right(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[FRONT_RIGHT_SWERVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_swerve_back_left(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[BACK_LEFT_SWERVE] = msg->data;
} 
void Joystick_Receiver_Node::receive_swerve_back_right(const std_msgs::Float64::ConstPtr & msg){
    out_msg.data[BACK_RIGHT_SWERVE] = msg->data;
} 