#include "arm_conversion.h"


/* Program takes in a UDRF file using the robot state publisher
   and a joint state message detailing the positions of the two
   actuators on the arm and publishes all the angles of the arm joints 

   Created C-Squire, 16/03/16, chris.squire93@gmail.com
*/


//I'm going to do this slightly different, I'm going to create a hashtable that relates actuator positions
//to main arm angle, the other angles are kind of pointless, it'll be in arm_convserion.h

int main() 
{
    ros::Rate r(100); //100 hz, 1/100th second    
    conversion Converter;
    Converter.initialiseMap();
    while(ros::ok()) 
    {
        ros::spin();
        r.sleep();
    }
    
}

//this function will call a transform function, and then a publish function
void conversion::actuatorCallBack(const sensor_msgs::JointState::ConstPtr& actuators)
{
    if(actuators->name[0] == UPPER_STROKE_NAME) {
    	inPositions[0] = std::make_pair(actuators->name[0], actuators->position[0]);
    	inPositions[1] = std::make_pair(actuators->name[1], actuators->position[1]);
    } else {
        inPositions[0] = std::make_pair(actuators->name[1], actuators->position[1]);
        inPositions[1] = std::make_pair(actuators->name[0], actuators->position[0]);
    }
    performTransform();
}

void conversion::performTransform() {
   
    //idea:
    //work out the closet predefined position
    //(Can do with modulus) and then:
    //initially, snap it to position
    //eventually assume linearity and determine position 

    //find first closest
    double extent1 = floor(inPositions[0].second/RESOLUTION); //gives an index
    double extent2 = floor(inPositions[1].second/RESOLUTION);

 
    //1m = 100cm
    //1cm = 10mm
    //1m = 1000mm 
    if(fmod(inPositions[0].second, RESOLUTION) > 0.0025) {
        extent1 = (extent1+1)*RESOLUTION;
    } else {
        extent1 = extent1*RESOLUTION;
    }

    if(fmod(inPositions[1].second,RESOLUTION) > 0.0025) {
        extent2 = (extent2+1)*RESOLUTION;
    } else {
        extent2 = extent2*RESOLUTION;
    }


    std::pair<double,double> link_angles = position_map.at(std::make_pair<double, double>(extent1, extent2)); 
    
    //pass angle into a joint state message & send it on
    angle_msgs.name.push_back("arm_base_link_joint");
    angle_msgs.name.push_back("arm_link_link_joint");
    angle_msgs.position.push_back(link_angles.first);
    angle_msgs.position.push_back(link_angles.second);

    angle_msgs.velocity.push_back(0.0);
    angle_msgs.velocity.push_back(0.0);

    angle_msgs.effort.push_back(0.0);    
    angle_msgs.effort.push_back(0.0);
   
    js_pub.publish(angle_msgs); 
    ros::Subscriber actuator_sub;
    ros::Publisher js_pub;

}

void conversion::initialiseMap() {
   
    int index = 0;
    double upper_stroke = 0.0;
    double lower_stroke = 0.0;
    
    for(double upper_stroke = 0.0; upper_stroke < UPPER_ACTUATOR_LIMIT; upper_stroke+=RESOLUTION) {
        for(double lower_stroke = 0.0; lower_stroke < LOWER_ACTUATOR_LIMIT; lower_stroke+=RESOLUTION) {
            std::pair<double, double> strokes = std::make_pair(upper_stroke, lower_stroke);
            std::pair<double, double> angles = std::make_pair(angle_list[0][index], angle_list[1][index]);
            position_map.insert(std::make_pair<std::pair<double, double>, std::pair<double, double> >(strokes, angles));
        }
    }

}
