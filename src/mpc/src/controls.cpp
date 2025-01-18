#include "controls.h"

NLPDController::NLPDController() {
    Kp = Eigen::Vector3d(8, 8, 12).asDiagonal();
    Kd = Eigen::Vector3d(5.5, 5.5, 8).asDiagonal();
    K_R = Eigen::Vector3d(100, 100, 10).asDiagonal();
    K_w = Eigen::Vector3d(20, 20, 10).asDiagonal();
    
    
    mass = 0.030;  // kg
    g = 9.81;      // m/s^2
    arm_length = 0.046;  // m
    k_thrust = 2.3e-8;
    k_drag = 7.8e-11;
    rotor_speed_min = 0;
    rotor_speed_max = 2500;
    
    inertia << 1.43e-5, 0, 0,
                0, 1.43e-5, 0,
                0, 0, 2.89e-5;
}

Eigen::Vector4d NLPDController::control(const DesiredState& desired_state, const State& current_state) {
    Eigen::Vector3d error_pos = current_state.position - desired_state.position;  
    Eigen::Vector3d error_vel = current_state.velocity - desired_state.velocity;

    RCLCPP_INFO(rclcpp::get_logger("pd_controller"),
        "Position error: (%f, %f, %f), Velocity error: (%f, %f, %f)",
        error_pos.x(), error_pos.y(), error_pos.z(),
        error_vel.x(), error_vel.y(), error_vel.z());

    Eigen::Vector3d rdd_des = desired_state.acceleration - Kd * error_vel - Kp * error_pos;
    std::cout << "desired_state.acceleration:" << desired_state.acceleration << std::endl;

    RCLCPP_INFO(rclcpp::get_logger("pd_controller"),
        "rdd_des: (%f, %f, %f)",
        rdd_des.x(), rdd_des.y(), rdd_des.z());
    
    Eigen::Vector3d F_des = mass * rdd_des + Eigen::Vector3d(0, 0, mass * g);
    
    RCLCPP_INFO(rclcpp::get_logger("pd_controller"),
        "F_des: (%f, %f, %f)",
        F_des.x(), F_des.y(), F_des.z());
    
    Eigen::Matrix3d R = Eigen::Quaterniond(
        current_state.quaternion[3],  // w
        current_state.quaternion[0],  // i
        current_state.quaternion[1],  // j
        current_state.quaternion[2]   // k
    ).toRotationMatrix();
    
    Eigen::Vector3d b3 = R.col(2);
    
    double u1 = b3.dot(F_des);
    RCLCPP_INFO(rclcpp::get_logger("pd_controller"),
    "u1 (thrust): %f", u1);
    
    Eigen::Vector3d b3_des = F_des.normalized();
    Eigen::Vector3d a_psi(
        cos(desired_state.yaw),
        sin(desired_state.yaw),
        0
    );
    Eigen::Vector3d b2_des = b3_des.cross(a_psi).normalized();
    Eigen::Vector3d b1_des = b2_des.cross(b3_des);
    Eigen::Matrix3d R_des;
    R_des.col(0) = b1_des;
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;
    
    Eigen::Matrix3d error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d e_R(-error_matrix(1,2), error_matrix(0,2), -error_matrix(0,1));
    Eigen::Vector3d u2 = inertia * (-K_R * e_R - K_w * current_state.angular_velocity);
    
    double gamma = k_drag / k_thrust;
    Eigen::Matrix4d cof;
    cof << 1, 1, 1, 1,
        0, arm_length, 0, -arm_length,
        -arm_length, 0, arm_length, 0,
        gamma, -gamma, gamma, -gamma;
    
    Eigen::Vector4d u(u1, u2[0], u2[1], u2[2]);
    Eigen::Vector4d F = cof.inverse() * u;
    Eigen::Vector4d motor_speeds = Eigen::Vector4d::Zero();
    for (int i = 0; i < 4; i++) {
        if (F[i] < 0) {
            F[i] = 0;
            motor_speeds[i] = rotor_speed_max;
        } else {
            motor_speeds[i] = std::sqrt(F[i] / k_thrust);
            if (motor_speeds[i] > rotor_speed_max) {
                motor_speeds[i] = rotor_speed_max;
            }
        }
    }
    return motor_speeds;
}