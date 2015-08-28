#include "SensorFusion.h"
#include <cmath>
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#define ACC_WEIGHT 0.05 
#define MAG_WEIGHT 0.0 
#define SYDNEY 12.33
#define POLAND 4.48
#define GYRO_HZ 5
int
main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_sensor_fusion_node");
    
    SensorFusion p;
    p.spin();

    return EXIT_SUCCESS;   
}


SensorFusion::SensorFusion() {
    //set up subs and pub
    subMag = node.subscribe("/mag", 5, &SensorFusion::receiveMag, this);
    
    subAccel = node.subscribe("/acc", 5, &SensorFusion::receiveAccel, this);
    subGyro = node.subscribe("/gyro", 5, &SensorFusion::receiveGyro, this);
    pub = node.advertise<owr_messages::heading>("/owr/heading",10);
    
}

void 
SensorFusion::receiveMag(const geometry_msgs::Vector3::ConstPtr& _mag) {
    allThree |= 0x1;
    mag.x = _mag->x;
    mag.y = _mag->y;
    mag.z = _mag->z;
    if(allThree == 0x7) fuseData();
}

void 
SensorFusion::receiveAccel(const geometry_msgs::Vector3::ConstPtr& _acc) {
    allThree |= 0x2;
    acc.x = _acc->x;
    acc.y = _acc->y;
    acc.z = _acc->z;
    if(allThree == 0x7) fuseData();
}

void 
SensorFusion::receiveGyro(const geometry_msgs::Vector3::ConstPtr& _gyro) {
    allThree |= 0x4;
    gyro.x = _gyro->x/GYRO_HZ;
    gyro.y = _gyro->y/GYRO_HZ;
    gyro.z = _gyro->z/GYRO_HZ;
    if(allThree == 0x7) fuseData();
}

void
SensorFusion::fuseData() {
    allThree = 0;
    // accel and mag
    // normalize data
    vector_normalize(mag);
    vector_normalize(acc);
    // calc `correction vector
    geometry_msgs::Vector3 corMag;
    vector_cross(dcm[2],mag,corMag);

    geometry_msgs::Vector3 corAcc;
    vector_cross(dcm[3],acc,corAcc);
    // gyro
    // avg the shit
    geometry_msgs::Vector3 weightedAvg;
    weightedAvg.x = (gyro.x + ACC_WEIGHT * corAcc.x * MAG_WEIGHT * corMag.x) / (1 + ACC_WEIGHT + MAG_WEIGHT);
    weightedAvg.y = (gyro.y + ACC_WEIGHT * corAcc.y * MAG_WEIGHT * corMag.y) / (1 + ACC_WEIGHT + MAG_WEIGHT);
    weightedAvg.z = (gyro.z + ACC_WEIGHT * corAcc.z * MAG_WEIGHT * corMag.z) / (1 + ACC_WEIGHT + MAG_WEIGHT);
    DCM_rotate(dcm, weightedAvg);
    // convert to roll pitch and yaw
    roll = radToDeg(-asin(dcm[2].x));
    pitch = radToDeg(atan2(dcm[2].y, dcm[2].z));
    yaw = radToDeg(atan2(dcm[1].x,dcm[0].x));
    //heading
    heading = radToDeg(atan2(mag.x,mag.y)) - SYDNEY;
    
    //publish stuff
    
}

double
SensorFusion::vector_mod(geometry_msgs::Vector3 v) {
    return sqrt( pow(v.x,2) + pow(v.y,2) + pow(v.z,2) );
}

void 
SensorFusion::vector_normalize(geometry_msgs::Vector3 v) {
    double length = vector_mod (v);
    v.x /= length;
    v.y /= length;
    v.z /= length;
}

void 
SensorFusion::vector_cross(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2, geometry_msgs::Vector3 vd) { 
    vd.x = v1.y * v2.z - v1.z * v2.y;   
    vd.y = v1.z * v2.x - v1.x * v2.z;   
    vd.z = v1.x * v2.y - v1.y * v2.x;   
}

double 
SensorFusion::vector_dot(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

void 
SensorFusion::vector_scale(double s, geometry_msgs::Vector3 v, geometry_msgs::Vector3 vd) {
    vd.x = s * v.x;
    vd.y = s * v.y;
    vd.z = s * v.z;
}

void 
SensorFusion::vector_add(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2, geometry_msgs::Vector3 vd) {
    vd.x = v1.x + v2.x;
    vd.y = v1.y + v2.y;
    vd.z = v1.z + v2.z;

}

void 
SensorFusion::DCM_othonormalize(geometry_msgs::Vector3 dcm[3]) {
   double error = vector_dot(dcm[0], dcm[1]);
   geometry_msgs::Vector3 delta[2];
   vector_scale(-error/2, dcm[1], delta[0]);
   vector_scale(-error/2, dcm[0], delta[1]);
   vector_add(dcm[0], delta[0], dcm[0]);
   vector_add(dcm[1], delta[1], dcm[1]);

   vector_cross(dcm[0], dcm[1], dcm[2]);
   vector_normalize(dcm[0]);
   vector_normalize(dcm[1]);
   vector_normalize(dcm[2]);
}

void 
SensorFusion::DCM_rotate (geometry_msgs::Vector3 dcm[3], geometry_msgs::Vector3 v) {
    int i;
    geometry_msgs::Vector3  dR;
    for (i = 0; i < 3; ++i) {
        vector_cross(v, dcm[i], dR);
        vector_add(dcm[i], dR, dcm[i]);
    }
    DCM_othonormalize(dcm);
}

void
SensorFusion::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}

double
radToDeg(double rad) {
    return (rad / M_PI) * 180;
}

