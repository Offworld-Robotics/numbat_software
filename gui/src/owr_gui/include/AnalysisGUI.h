/*
 * This manages input into the GUI from ROS
 * By Harry J.E Day for BlueSat OWR <Harry@dayfamilyweb.com>
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
//#include "../../devel/include/bluesat_owr_protobuf/battery_ros.h"
#include "site.h"

class ANALYSISGUI {

    public:
        ANALYSISGUI();
        void spin();
        void reciveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg);
        //void reciveSiteMsg();// pH, ultrasonic, etc
        void reciveVideoMsg(const sensor_msgs::Image::ConstPtr& msg);
        
    private:
        std::string coOrdList;
        ros::Subscriber gpsSub;
        //ros::Subscriber siteSub;
        ros::Subscriber videoSub;
        double latitude;
        double longitude;
        float altitude;
        float pH;
        float ultrasonic;
        float humidity;
        unsigned char *frame; // frame of video
        //void  (*updateConstants)UPDATE_CONST_FUNCTION_DEF;
};

