#ifndef _CONTROLS_H_
#define _CONTROLS_H_

#include "utils.h"
#include "quadrotor.h"
#include <casadi/casadi.hpp>
using namespace casadi;

class NLPDController {
private:
    double  kp_x,kp_y,kp_z,
            kd_x,kd_y,kd_z,
            kr_x, kr_y, kr_z,
            kw_x, kw_y, kw_z;
    Eigen::Matrix3d Kp, Kd, K_R, K_w;
public:
    NLPDController(rclcpp::Node* parent_node);
    Eigen::Vector4d control(const DesiredState& desired_state, const State& current_state);
    Eigen::Matrix3d inertia;
    double mass, g, arm_length, k_thrust, k_drag, rotor_speed_min, rotor_speed_max;
};

class QuadrotorMPC {
private:
    double mass, g, arm_length, k_thrust, k_drag, rotor_speed_min, rotor_speed_max,
    dt, Ixx, Iyy, Izz, N, pos_weight, vel_weight, att_weight, omega_weight, input_weight;
    Function solver;
    void setup_solver();
    Function create_dynamics();
public:
    QuadrotorMPC(rclcpp::Node* parent_node);
    Eigen::Vector4d solve(const State& current_state, const DesiredState& desired_state);

};
#endif