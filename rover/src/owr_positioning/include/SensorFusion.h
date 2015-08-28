#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include "owr_messages/heading.h"
class SensorFusion{
    public:
        SensorFusion();
        void spin();
    protected:
        void receiveMag(const geometry_msgs::Vector3::ConstPtr&);
        void receiveAccel(const geometry_msgs::Vector3::ConstPtr&);
        void receiveGyro(const geometry_msgs::Vector3::ConstPtr&);
        void fuseData();
        ros::Subscriber subMag;
        ros::Subscriber subAccel;
        ros::Subscriber subGyro;
        ros::Publisher pub;

        geometry_msgs::Vector3 mag;
        geometry_msgs::Vector3 acc;
        geometry_msgs::Vector3 gyro;

        geometry_msgs::Vector3 dcm[3];

        double roll;
        double pitch;
        double yaw;
        double heading;

        unsigned char allThree; 
    private:
        ros::NodeHandle node;
        
        double vector_mod(geometry_msgs::Vector3);
        void vector_normalize(geometry_msgs::Vector3);
        void vector_cross(geometry_msgs::Vector3, geometry_msgs::Vector3, geometry_msgs::Vector3);
        double vector_dot(geometry_msgs::Vector3, geometry_msgs::Vector3); 
        void vector_scale(double ,geometry_msgs::Vector3, geometry_msgs::Vector3);
        void vector_add(geometry_msgs::Vector3, geometry_msgs::Vector3, geometry_msgs::Vector3); 
        void DCM_othonormalize(geometry_msgs::Vector3 [3]);
        void DCM_rotate (geometry_msgs::Vector3 [3],geometry_msgs::Vector3);
};

double radToDeg (double);
