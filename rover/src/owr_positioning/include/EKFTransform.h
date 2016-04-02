/* 
 * EKF transform node for GPS coordinate data
 * Author: Elliott Smith for bluesat OWR
 * Date: 19/03/2016
*/


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <list>
#include <cmath>
#include <stdio.h>
#include <sensor_msgs/NavSatFix.h>

#define TOPIC "/owr/ekf_transformation"

class EKFTransform {
    public:
	    EKFTransform();
	    void spin();
    protected:
	    void callback(const boost::shared_ptr<sensor_msg::NavSatFix> &message);
	    ros::Publisher publisher;
	    ros::Subscriber subscriber;
    private:
        ros::NodeHandle node;
};



		
