#include "quadrotor.h"

Quadrotor::Quadrotor() {

    mass = 0.030;  // kg
    Ixx = 1.43e-5;  // kg*m^2
    Iyy = 1.43e-5;  // kg*m^2
    Izz = 2.89e-5;  // kg*m^2
    arm_length = 0.046;  // meters
    rotor_speed_min = 0;  // rad/s
    rotor_speed_max = 2500;  
    k_thrust = 2.3e-8;  // N/(rad/s)^2
    k_drag = 7.8e-11;   // Nm/(rad/s)^2
    g = 9.81;  // m/s^2
    dt = 0.01; // Time step
    
    
    inertia = Eigen::Matrix3d::Zero();
    inertia(0,0) = Ixx;
    inertia(1,1) = Iyy;
    inertia(2,2) = Izz;
    inv_inertia = inertia.inverse();

    double k = k_drag/k_thrust;
    double L = arm_length;
    to_TM << 1,  1,  1,  1,
            0,  L,  0, -L,
            -L,  0,  L,  0,
            k, -k,  k, -k;
}

State Quadrotor::reset(const Eigen::Vector3d& position,
            double yaw, double pitch, double roll) {
    State state;
    
    // Initialize position
    state.position = position;
    state.velocity = Eigen::Vector3d::Zero();
    state.angular_velocity = Eigen::Vector3d::Zero();
    
    // Convert Euler angles to quaternion 
    // Note: Using [i,j,k,w] 
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
                        
    state.quaternion = Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
    
    current_state = state;
    return state;
}

State Quadrotor::step(const Eigen::Vector4d& cmd_rotor_speeds) {
    // Clip rotor speeds
    Eigen::Vector4d rotor_speeds = cmd_rotor_speeds.cwiseMax(rotor_speed_min).cwiseMin(rotor_speed_max);
    
    Eigen::Vector4d rotor_thrusts = k_thrust * rotor_speeds.array().square().matrix();
    Eigen::Vector4d TM = to_TM * rotor_thrusts;
    
    double T = TM[0];  // Thrust
    Eigen::Vector3d M(TM[1], TM[2], TM[3]);  // Moments
    
    current_state = rk4_step(current_state, T, M, dt);
    
    // Normalize quaternion
    current_state.quaternion.normalize();
    
    return current_state;
}
    
State Quadrotor::rk4_step(const State& state, double thrust, const Eigen::Vector3d& moment, double dt) {
    auto k1 = state_derivative(state, thrust, moment);
    auto k2 = state_derivative(state_plus_derivative(state, k1, dt/2), thrust, moment);
    auto k3 = state_derivative(state_plus_derivative(state, k2, dt/2), thrust, moment);
    auto k4 = state_derivative(state_plus_derivative(state, k3, dt), thrust, moment);
    
    State new_state;
    new_state.position = state.position + dt/6 * (k1.position + 2*k2.position + 2*k3.position + k4.position);
    new_state.velocity = state.velocity + dt/6 * (k1.velocity + 2*k2.velocity + 2*k3.velocity + k4.velocity);
    new_state.quaternion = state.quaternion + dt/6 * (k1.quaternion + 2*k2.quaternion + 2*k3.quaternion + k4.quaternion);
    new_state.angular_velocity = state.angular_velocity + dt/6 * 
        (k1.angular_velocity + 2*k2.angular_velocity + 2*k3.angular_velocity + k4.angular_velocity);
    
    return new_state;
}

State Quadrotor::state_derivative(const State& state, double thrust, const Eigen::Vector3d& moment) {
    State derivative;
    
    // Position derivative
    derivative.position = state.velocity;
    
    // Velocity derivative (corrected to match Python)
    Eigen::Vector3d gravity(0, 0, -g);
    Eigen::Vector3d force = gravity * mass + rotate_k(state.quaternion) * thrust;
    derivative.velocity = force / mass;
    
    // Orientation derivative using corrected quaternion order
    derivative.quaternion = quatDot(state.quaternion, state.angular_velocity);
    
    // Angular velocity derivative
    Eigen::Vector3d omega = state.angular_velocity;
    derivative.angular_velocity = inv_inertia * 
        (moment - omega.cross(inertia * omega));
    
    return derivative;
}

State Quadrotor::state_plus_derivative(const State& state, const State& derivative, double dt) {
    State result;
    result.position = state.position + derivative.position * dt;
    result.velocity = state.velocity + derivative.velocity * dt;
    result.quaternion = state.quaternion + derivative.quaternion * dt;
    result.angular_velocity = state.angular_velocity + derivative.angular_velocity * dt;
    return result;
}

Eigen::Vector3d Quadrotor::rotate_k(const Eigen::Vector4d& q) {
    return Eigen::Vector3d(
        2 * (q[0] * q[2] + q[1] * q[3]),
        2 * (q[1] * q[2] - q[0] * q[3]),
        1 - 2 * (q[0] * q[0] + q[1] * q[1])
    );
}

Eigen::Vector4d Quadrotor::quatDot(const Eigen::Vector4d& quat, const Eigen::Vector3d& omega) {
    
    Eigen::Matrix<double, 3, 4> G;
    double q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    
    G << q3,  q2, -q1, -q0,
         -q2,  q3,  q0, -q1,
          q1, -q0,  q3, -q2;
    
    Eigen::Vector4d quatDot = 0.5 * G.transpose() * omega;
    
    double quatErr = quat.dot(quat) - 1;
    Eigen::Vector4d quatErrGrad = 2 * quat;
    quatDot = quatDot - quatErr * quatErrGrad;
    
    return quatDot;
}