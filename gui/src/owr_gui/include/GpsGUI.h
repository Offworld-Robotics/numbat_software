/*
 * This manages input into the GUI from ROS
 * By Harry J.E Day for BlueSat OWR <Harry@dayfamilyweb.com>
 */

#include "comms.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "../../devel/include/bluesat_owr_protobuf/battery_ros.h"
#include  "OwrGui.h"

class GPSGUI {

    public:
        GPSGUI(OwrGui*  gui);
        void spin();
        void reciveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void reciveBatteryMsg(const bluesat_owr_protobuf::battery_ros::ConstPtr& msg);
        
    private:
        std::string coOrdList;
        OwrGui* gui;
        ros::Subscriber gpsSub;
        ros::Subscriber batterySub;
        ListNode list;
        ListNode end;
        float battery;
        float signal;
        float tiltX;
        float tiltY;
        float ultrasonic;
        vector2D target;
        //void  (*updateConstants)UPDATE_CONST_FUNCTION_DEF;
};

