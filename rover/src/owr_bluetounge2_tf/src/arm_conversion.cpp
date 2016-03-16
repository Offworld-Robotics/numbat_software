#include "arm_conversion.h"


/* Program takes in a UDRF file using the robot state publisher
   and a joint state message detailing the positions of the two
   actuators on the arm and publishes all the angles of the arm joints 

   Created C-Squire, 16/03/16, chris.squire93@gmail.com
*/


int main() 
{
    ros::init(argc, argv, "arm_conversion");
    ros::NodeHandle n;
    ros::Subscriber actuator_sub = n.subsrciber("actuators", 1, actuatorCallBack);
    ros::Publisher js_pub = n.advertise<sensor_msgs::joint_state>("arm_js", 1);

    while(ros::ok()) 
    {

    }
}

//this function will call a transform function, and then a publish function
void actuatorCallBack(const sensor_msgs::joint_state::ConstPtr& actuators)
{
    std::vector<double> positions;
    positions.resize(2);    
    if(actuators->name != " ") {
        positions[0] = msg->position[0];
        positions[1] = msg->position[1];    
    } else {
        positions[1] = msg->position[0];
        positions[0] = msg->position[1];
    }
    performTransform(positions)
}


