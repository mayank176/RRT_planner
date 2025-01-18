#ifndef _CONTROLS_H_
#define _CONTROLS_H_

#include "utils.h"
#include "quadrotor.h"

class NLPDController {
private:
    Eigen::Matrix3d Kp, Kd, K_R, K_w;
public:
    NLPDController();
    Eigen::Vector4d control(const DesiredState& desired_state, const State& current_state);
    Eigen::Matrix3d inertia;
    double mass, g, arm_length, k_thrust, k_drag, rotor_speed_min, rotor_speed_max;
};
#endif