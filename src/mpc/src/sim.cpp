#include "sim.h"

double QuadrotorEnv::sqnorm_err(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d diff = v1 - v2;
    return diff.squaredNorm();
}

QuadrotorEnv::QuadrotorEnv() : quadrotor() {
    // Initialize environment limits
    pos_limits = 10.0;  // Position limits in meters
    vel_limits = 5.0;   // Velocity limits in m/s
    angle_limits = M_PI / 6;  // Angular limits in radians
    rate_limits = 2.0;  // Angular rate limits in rad/s
    goal_threshold = 0.1; // Distance threshold for reaching goal
}

State QuadrotorEnv::reset(const Eigen::Vector3d& position) {
    current_state = quadrotor.reset(position);
    time_step = 0;
    return current_state;
}

QuadrotorEnv::EnvState QuadrotorEnv::step(const Eigen::Vector4d& action, const Eigen::Vector3d& goal_pos) {
    // Step the quadrotor dynamics
    current_state = quadrotor.step(action);
    time_step++;

    // Calculate reward and done condition
    double reward = calculateReward();
    bool done = isTerminal();

    double end_pos_error = sqnorm_err(goal_pos, current_state.position);
    bool reached_goal = end_pos_error < goal_threshold;

    // Pack additional info
    std::map<std::string, double> info;
    info["time_step"] = time_step;
    info["distance_to_goal"] = std::sqrt(end_pos_error);
    info["end_pos_error"] = end_pos_error;

    return {current_state, reward, done, reached_goal, info};
}


double QuadrotorEnv::calculateReward() {
    
    double distance_penalty = calculateDistanceToGoal();
    double velocity_penalty = current_state.velocity.norm();
    double orientation_penalty = 1.0 - current_state.quaternion[3]; 
    
    return -(distance_penalty + 0.1 * velocity_penalty + 0.1 * orientation_penalty);
}


Eigen::Vector3d QuadrotorEnv::getEulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler;
    
    // Get Euler angles using atan2 
    euler[0] = atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));  // Roll
    euler[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));  // Pitch
    euler[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));  // Yaw
    
    return euler * 180.0 / M_PI;  //degrees
}

bool QuadrotorEnv::isTerminal() {
    // std::cout << "\nChecking termination conditions:" << std::endl;
    // std::cout << "Current velocities: " << current_state.velocity.transpose() 
    //           << " (limit: ±" << vel_limits << ")" << std::endl;
    
    // Convert quaternion to Euler angles
    Eigen::Quaterniond q(current_state.quaternion[3],
                        current_state.quaternion[0],
                        current_state.quaternion[1],
                        current_state.quaternion[2]);
    Eigen::Vector3d euler = getEulerAngles(q);
    // std::cout << "Current angles (deg): " << euler.transpose() 
    //           << " (limit: ±" << angle_limits * 180/M_PI << ")" << std::endl;
    
    std::cout << "Current angular rates: " << current_state.angular_velocity.transpose() 
                << " (limit: ±" << rate_limits << ")" << std::endl;

    for (int i = 0; i < 3; i++) {
        if (std::abs(current_state.velocity[i]) > vel_limits) {
            std::cout << "Terminating: Velocity limit exceeded on axis " << i << std::endl;
            return true;
        }
    }

    for (int i = 0; i < 3; i++) {
        if (std::abs(euler[i]) > angle_limits * 180/M_PI) {
            std::cout << "Terminating: Angle limit exceeded on axis " << i << std::endl;
            return true;
        }
    }

    if (current_state.angular_velocity.norm() > rate_limits) {
        std::cout << "Terminating: Angular rate limit exceeded" << std::endl;
        return true;
    }

    return false;
}

double QuadrotorEnv::calculateDistanceToGoal() { //need to change
    
    Eigen::Vector3d goal(2, 2, 2);  // Example goal
    return (current_state.position - goal).norm();
}

Trajectory::Trajectory(double dt) : t(0), dt(dt) {}

std::string Trajectory::getName() const { return name; }

Eigen::Vector3d Trajectory::tj_from_line(const Eigen::Vector3d& start_pos, 
                                    const Eigen::Vector3d& end_pos, 
                                    double time_ttl, 
                                    double t_c) {
    Eigen::Vector3d v_max = (end_pos - start_pos) * 2.0 / time_ttl;
    Eigen::Vector3d pos, vel;
    
    if (t_c >= 0 && t_c < time_ttl/2) {
        vel = v_max * t_c / (time_ttl/2);
        pos = start_pos + t_c * vel/2;
    } else {
        vel = v_max * (time_ttl - t_c) / (time_ttl/2);
        pos = end_pos - (time_ttl - t_c) * vel/2;
    }
    return pos;
}


HoverTrajectory::HoverTrajectory(double dt, double hoverHeight, double T) 
    : Trajectory(dt), hoverHeight(hoverHeight), T(T) {}

DesiredState HoverTrajectory::getDesState(double t) {
    this->t = t;
    Eigen::Vector3d start_pos(0, 0, 0);
    Eigen::Vector3d end_pos(0, 0, hoverHeight);

    DesiredState state;
    if (t > T) {
        state.position = end_pos;
        state.velocity = Eigen::Vector3d::Zero();
        state.acceleration = Eigen::Vector3d::Zero();
        state.yaw = 0;
        state.yaw_dot = 0;
        return state;
    }

    state.position = tj_from_line(start_pos, end_pos, T, t);
    Eigen::Vector3d nextPos = tj_from_line(start_pos, end_pos, T, t + dt);
    Eigen::Vector3d nextNextPos = tj_from_line(start_pos, end_pos, T, t + 2*dt);

    state.velocity = (nextPos - state.position) / dt;
    Eigen::Vector3d nextVel = (nextNextPos - nextPos) / dt;
    state.acceleration = (nextVel - state.velocity) / dt;

    // Limit max velocity and acceleration
    for (int i = 0; i < 3; i++) {
        state.velocity[i] = std::max(-1.0, std::min(1.0, state.velocity[i]));
        state.acceleration[i] = std::max(-1.0, std::min(1.0, state.acceleration[i]));
    }

    state.yaw = 0;
    state.yaw_dot = 0;
    return state;
}

std::vector<double> CustomTrajectory::divideTimeFromPath(const std::vector<Eigen::Vector3d>& path, double T) {
    size_t totalWaypoints = path.size();
    std::vector<double> distanceList;
    std::vector<double> tArr(totalWaypoints, 0.0);

    // Calculate distances between consecutive waypoints
    double totalDistance = 0.0;
    for (size_t i = 0; i < totalWaypoints - 1; i++) {
        double distance = (path[i+1] - path[i]).norm();
        distanceList.push_back(distance);
        totalDistance += distance;
    }

    // Distribute time proportionally to distances
    double sumDist = 0.0;
    for (size_t i = 0; i < totalWaypoints - 1; i++) {
        sumDist += distanceList[i];
        tArr[i + 1] = (sumDist / totalDistance) * T;  // tArr[0] stays 0
    }

    for (size_t i = 0; i < tArr.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("custom_trajectory"), 
            "Waypoint %zu time: %f, position: (%f, %f, %f)", 
            i, tArr[i], path[i].x(), path[i].y(), path[i].z());
    }

        return tArr;
}


CustomTrajectory::CustomTrajectory(double dt, const std::vector<Eigen::Vector3d>& trajectory, double time, 
                const std::string& name)
    : Trajectory(dt)
    , T(time)
    , traj(trajectory)
    , waypointIndex(0) 
{
    this->name = name;
    tArr = divideTimeFromPath(traj, T);
}

DesiredState CustomTrajectory::getDesState(double t) {
    this->t = t;
    DesiredState state;

    if (t == 0.0) {  //start position
        state.position = traj[0];
        state.velocity = Eigen::Vector3d::Zero();
        state.acceleration = Eigen::Vector3d::Zero();
        state.yaw = 0;
        state.yaw_dot = 0;
        return state;
    }

    if (t < T) {
        if (t > tArr[waypointIndex]) {
            waypointIndex++;
        }

        if (waypointIndex >= traj.size() - 1) {
            state.velocity = Eigen::Vector3d::Zero();
            state.acceleration = Eigen::Vector3d::Zero();
            state.position = traj.back();
            state.yaw = 0;
            state.yaw_dot = 0;
            return state;
        }

        Eigen::Vector3d start_pos = traj[waypointIndex];
        Eigen::Vector3d end_pos = traj[waypointIndex + 1];
        double t_c = t - (tArr[waypointIndex + 1] - tArr[waypointIndex]);

        state.position = tj_from_line(start_pos, end_pos, T, t_c);
        Eigen::Vector3d nextPos = tj_from_line(start_pos, end_pos, T, t_c + dt);
        Eigen::Vector3d nextNextPos = tj_from_line(start_pos, end_pos, T, t_c + 2*dt);

        state.velocity = (nextPos - state.position) / dt;
        Eigen::Vector3d nextVel = (nextNextPos - nextPos) / dt;
        state.acceleration = (nextVel - state.velocity) / dt;

        for (int i = 0; i < 3; i++) {
            state.velocity[i] = std::max(-1.0, std::min(1.0, state.velocity[i]));
            state.acceleration[i] = std::max(-1.0, std::min(1.0, state.acceleration[i]));
        }

    } else {  // t >= T
        state.position = traj.back();
        state.velocity = Eigen::Vector3d::Zero();
        state.acceleration = Eigen::Vector3d::Zero();
    }

    state.yaw = 0;
    state.yaw_dot = 0;
    return state;
}

void QuadrotorController::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", msg->poses.size());

// Convert path message to vector of waypoints
    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& pose : msg->poses) {
        waypoints.push_back(Eigen::Vector3d(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ));
    }
    
    // Create a custom trajectory from the RRT path
    trajectory_ = std::make_unique<CustomTrajectory>(0.01, waypoints);

    // Reset simulation state
    if (!waypoints.empty()) {
        quad_env_->reset(waypoints[0]);
    }

    // Store path for visualization
    planned_path_ = waypoints;
    executed_path_.clear();
    path_received_ = true;
    current_waypoint_ = 0;

}    

visualization_msgs::msg::Marker QuadrotorController::create_executed_path_marker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "executed_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;  
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    for (const auto& point : executed_path_) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
    }
    
    return marker;
}

visualization_msgs::msg::Marker QuadrotorController::create_quadrotor_marker(const State& state) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "quadrotor";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = state.position.x();
    marker.pose.position.y = state.position.y();
    marker.pose.position.z = state.position.z();
    
    marker.pose.orientation.x = state.quaternion[0];
    marker.pose.orientation.y = state.quaternion[1];
    marker.pose.orientation.z = state.quaternion[2];
    marker.pose.orientation.w = state.quaternion[3];
    
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.1;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    return marker;
}

    void QuadrotorController::simulate_step() {
    if (!path_received_ || !trajectory_) {
        return;
    }

    static bool initialized = false;
    static double t = 0.0;
    static double simulation_time = 15.0;
    static State current_state;
    static DesiredState end_state;
    static DesiredState start_state;

    if (!initialized) {
        start_state = trajectory_->getDesState(t);
        RCLCPP_INFO(this->get_logger(), "Start state: %f, %f, %f, %f", start_state.position.x(), start_state.position.y(), start_state.position.z(), t);
        end_state = trajectory_->getDesState(simulation_time);
        current_state = quad_env_->reset(start_state.position);
        initialized = true;
    }

    if (t >= simulation_time) {
        RCLCPP_INFO(this->get_logger(), "Simulation complete");
        return;
    }
    
    auto desired_traj_state = trajectory_->getDesState(t);
    // std::cout << "desired_traj_state_ct: " << desired_traj_state.position.transpose() << std::endl;

    auto motor_speeds = controller_->control(desired_traj_state, current_state);

    
    RCLCPP_INFO(this->get_logger(), "Motor speeds: %f, %f, %f, %f",
            motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);


    auto env_state = quad_env_->step(motor_speeds, end_state.position);//desired_traj_state.position); // or use end_state.position check
    
    current_state = env_state.state;
    // std::cout << "current state: " << current_state.position.transpose() << std::endl;
    // Store executed path
    executed_path_.push_back(current_state.position);
    planned_path_.push_back(desired_traj_state.position);
    
    // Publish visualization
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(create_executed_path_marker());
    marker_array.markers.push_back(create_quadrotor_marker(current_state));
    viz_publisher_->publish(marker_array);
    
    t += 0.01; 
}

QuadrotorController::QuadrotorController() 
    : Node("quadrotor_controller"),
        current_waypoint_(0),
        path_received_(false),
        waypoint_threshold_(0.5)
{
    viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "quadrotor_visualization", 10);
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "rrt_path", 10,
        std::bind(&QuadrotorController::path_callback, this, std::placeholders::_1));
    
    quad_env_ = std::make_unique<QuadrotorEnv>();
    controller_ = std::make_unique<NLPDController>();
    
    using namespace std::chrono_literals;
    sim_timer_ = this->create_wall_timer(
        10ms, std::bind(&QuadrotorController::simulate_step, this));
    
    RCLCPP_INFO(this->get_logger(), "Quadrotor controller node initialized");
}