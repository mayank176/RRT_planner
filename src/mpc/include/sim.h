#ifndef _SIM_H_
#define _SIM_H_

#include "utils.h"
#include "quadrotor.h"
#include "controls.h"


class QuadrotorEnv {
private:
    Quadrotor quadrotor;
    State current_state;
    int time_step;
    double goal_threshold;
    
    double pos_limits;
    double vel_limits;
    double angle_limits;
    double rate_limits;

    double calculateReward(const Eigen::Vector3d& goal_pos);
    Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond& q);
    bool isTerminal();
    double calculateDistanceToGoal(const Eigen::Vector3d& goal_pos);
    double sqnorm_err(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

public:
    struct EnvState {
        State state;
        double reward;
        bool done;
        bool reached_goal;
        std::map<std::string, double> info;
    };
    QuadrotorEnv();
    State reset(const Eigen::Vector3d& position = Eigen::Vector3d::Zero());
    EnvState step(const Eigen::Vector4d& action, const Eigen::Vector3d& goal_pos);
};


class Trajectory {
protected:
    double t;
    double dt;
    double T;
    std::string name;

    static Eigen::Vector3d tj_from_line(const Eigen::Vector3d& start_pos, 
                                      const Eigen::Vector3d& end_pos, 
                                      double time_ttl, 
                                      double t_c);
public:
    Trajectory(double dt, double simulation_time) 
        : t(0), dt(dt), T(simulation_time) {}
    std::string getName() const;
    virtual DesiredState getDesState(double t) = 0;
};


class HoverTrajectory : public Trajectory {
private:
    double hoverHeight;
    double T;
public:
    HoverTrajectory(double dt, double T,  double hover_height = 2.0) 
        : Trajectory(dt, T), hoverHeight(hover_height) {}
    DesiredState getDesState(double t) override;
};

class CustomTrajectory : public Trajectory {
private:
    double T;
    std::vector<Eigen::Vector3d> traj;
    std::vector<double> tArr;
    size_t waypointIndex;

    std::vector<double> divideTimeFromPath(const std::vector<Eigen::Vector3d>& path, double T);

public:
    CustomTrajectory(double dt, const std::vector<Eigen::Vector3d>& trajectory, double T,
                    const std::string& name = "Custom");
    DesiredState getDesState(double t) override;
};

class QuadrotorController : public rclcpp::Node {
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    
    std::unique_ptr<QuadrotorEnv> quad_env_;
    std::unique_ptr<NLPDController> controller_;
    // std::unique_ptr<QuadrotorMPC> controller_;
    std::unique_ptr<Trajectory> trajectory_;
    
    std::vector<Eigen::Vector3d> planned_path_;
    std::vector<Eigen::Vector3d> executed_path_;

    size_t current_waypoint_;
    bool path_received_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    visualization_msgs::msg::Marker create_executed_path_marker();
    visualization_msgs::msg::Marker create_quadrotor_marker(const State& state);
    void simulate_step();
public:
    double simulation_time;
    QuadrotorController();
};


#endif