#include "SensorFusion.h"
#include <cmath>

int main(int argc, char  argv) {
    ros::init(argc, argv, "owr_sensor_fusion__node");
    
    SensorFusion p;
    p.spin();

    return EXIT_SUCCESS;   
}


SensorFusion::SensorFusion() {
    //set up subs and pub
}

void 
SensorFusion::receiveMag() {

}

void 
SensorFusion::receiveAccel() {

}

void 
SensorFusion::receiveGyro() {

}

void
SensorFusion::fuseData() {
    // accel and mag
    // normalize data
    // calc correction vector
    // gyro
    // avg the shit
    // convert to roll pitch and yaw
}

double
vector_mod(geometry::Vector3 v) {
    return sqrt( pow(v->x,2) + pow(v->y,2) + pow(v->z,2) );
}

void 
vector_normalize(geometry::Vector3 v) {
    double length = vector_mod (v);
    v->x /= length;
    v->y /= length;
    v->z /= length;
}

void 
vector_cross(geometry::Vector3 v1, geometry::Vector3 v2, geometry::Vector3 vd) { 
    v->x = v1->y * v2->z - v1->z * v2->y;   
    v->y = v1->z * v2->x - v1->x * v2->z;   
    v->z = v1->x * v2->y - v1->y * v2->x;   
}

double 
vector_dot(geometry::Vector3 v1, geometry::Vector3 v2) {
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

void 
vector_scale(double s, geometry::Vector3 v, geometry::Vector3 vd) {
    vd->x = s v->x;
    vd->y = s v->y;
    vd->z = s v->z;
}

void 
vector_add(geometry::Vector3 v1, geometry::Vector3 v2, geometry::Vector3 vd) {
    vd->x = v1->x + v2->x;
    vd->y = v1->y + v2->y;
    vd->z = v1->z + v2->z;

}

void 
DCM_othonormalize(geometry::Vector3 dcm[3]) {
   double error = vector_dot(dcm[0], dcm[1];
   double delta[2][3];
   vector_scale(-err/2, dcm[1], delta[0]);
   vector_scale(-err/2, dcm[0], delta[1]);
   vector_add(dcm[0], delta[0], dcm[0]);
   vector_add(dcm[1], delta[1], dcm[1]);

   vector_cross(dcm[0], dcm[1], dcm[2]);
   vector_normalize(dcm[0]);
   vector_normalize(dcm[1]);
   vector_normalize(dcm[2]);
}

void 
DCM_rotate (geometry::Vector3 dcm[3], geometry::Vector3 v) {
    int i;
    geometry::Vector3  dR;
    for (i = 0; i < 3; ++i) {
        vector_cross(v, dcm[i], dR);
        vector_add(dcm[i], dR, dcm[i]);
    }
    dcm_orthonormalize(dcm);
}

void
SensorFusion::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
