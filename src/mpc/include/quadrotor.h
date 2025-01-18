#ifndef _QUADROTOR_H_
#define _QUADROTOR_H_

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include "utils.h"

class Quadrotor {
private:
    State current_state;
    double mass, Ixx, Iyy, Izz, arm_length;
    double rotor_speed_min, rotor_speed_max;
    double k_thrust, k_drag, g, dt;
    Eigen::Matrix3d inertia, inv_inertia;
    Eigen::Matrix4d to_TM;
    Eigen::Vector4d quatDot(const Eigen::Vector4d& quat, const Eigen::Vector3d& omega);

    State rk4_step(const State& state, double thrust, const Eigen::Vector3d& moment, double dt);
    State state_derivative(const State& state, double thrust, const Eigen::Vector3d& moment);
    State state_plus_derivative(const State& state, const State& derivative, double dt);
    static Eigen::Vector3d rotate_k(const Eigen::Vector4d& q);

public:
    Quadrotor();
    State reset(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
                double yaw = 0, double pitch = 0, double roll = 0);
    State step(const Eigen::Vector4d& cmd_rotor_speeds);
};

#endif