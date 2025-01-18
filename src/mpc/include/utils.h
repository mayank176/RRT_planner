#ifndef _UTILS_H_
#define _UTILS_H_

#include <eigen3/Eigen/Dense>

struct DesiredState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double yaw;
    double yaw_dot;
};

struct State {
    Eigen::Vector3d position;          // x, y, z position
    Eigen::Vector3d velocity;          // dx, dy, dz velocity
    Eigen::Vector4d quaternion;        // [i,j,k,w] quaternion orientation
    Eigen::Vector3d angular_velocity;  // wx, wy, wz angular velocity
    
    State() : 
        position(Eigen::Vector3d::Zero()),
        velocity(Eigen::Vector3d::Zero()),
        quaternion(Eigen::Vector4d(0, 0, 0, 1)), // Initialize to identity quaternion [0,0,0,1]
        angular_velocity(Eigen::Vector3d::Zero()) {}
    
};
#endif