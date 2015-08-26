#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

class DataFusion{
    public:
        DataFusion();
        void spin();
    protected:
        void receiveMag();
        void receiveAccel();
        void receiveGyro();
        ros::Subscriber subMag;
        ros::Subscriber subAccel;
        ros::Subscriber subGyro;
        ros::Publisher pub;

        geometry::Vector3 mag;
        geometry::Vector3 acc;
        geometry::Vector3 gyro;

    private:
        ros::NodeHandle node;

        double vector_mod(geometry::Vector3);
        void vector_normalize(geometry::Vector3);
        void vector_cross(geometry::Vector3, geometry::Vector3, geometry::Vector3);
        double vector_dot(geometry::Vector3, geometry::Vector3) 
        void vector_scale(double ,geometry::Vector3, geometry::Vector3);
        void vector_add(geometry::Vector3, geometry::Vector3, geometry::Vector3); 
        void DCM_othonormalize(geometry::Vector3 [3]);
        void DCM_rotate (geometry::Vector3 [3],geometry::Vector3);
}
