/*
 * Date Started: 4/02/2017
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: owr_mapping
 * ROS Package: owr_package_name
 * Purpose: Represents an map feature 
 */
#include <geometry_msgs/PoseStamped.h>

class AbstractFeature {
    public:
        virtual geometry_msgs::PoseStamped get_position() = 0;
};