#ifndef _RRT_VIS_H_
#define _RRT_VIS_H_  

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../../rrt/include/rrt.h"

class path_visualizer : public rclcpp::Node, public std::enable_shared_from_this<path_visualizer> {
private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        std::unique_ptr<RRT> rrt_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<Point> path_;
        bool planning_in_progress_;

        visualization_msgs::msg::Marker create_tree_marker();
        visualization_msgs::msg::MarkerArray create_obstacle_marker();
        visualization_msgs::msg::Marker create_path_marker();
        void visualize_and_plan();

public:
        path_visualizer();
};
#endif