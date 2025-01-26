#ifndef _RRT_H_
#define _RRT_H_

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Point {
    double x, y, z;
    Point() : x(0), y(0), z(0) {}
    Point(double x, double y, double z);
    double distance(const Point &other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
    }
};

struct rrtNode {
    Point position;
    rrtNode* parent;
    std::vector<rrtNode*> children;
    rrtNode(const Point &position, rrtNode* parent) : position(position), parent(parent) {}
};

struct Cuboid {
    
    Point min, max;
    Cuboid(Point min_, Point max_); 
    //check if a point is inside the cuboid
    bool point_intersect(const Point &point, double safety_margin) const;
    //check if a line intersects the cuboid
    bool line_intersect(const Point &start, const Point &end, double safety_margin) const;
};

class RRT : public rclcpp::Node {
public:
    Point min_bound, max_bound;
    Point start, goal;
    double step_size; //dist between a new node and the nearest node in the tree
    double safety_margin;
    int max_iterations;
    int env_num;
    std::vector<Cuboid> obstacles {};
    rrtNode* root {};
    std::vector<rrtNode*> nodes {};
    std::mt19937 rng {};
    std::uniform_real_distribution<double> dist_x {};
    std::uniform_real_distribution<double> dist_y {};
    std::uniform_real_distribution<double> dist_z {};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void add_obstacle(const Cuboid &obstacle);
    Point random_point();
    rrtNode* nearest_node(const Point &point);
    bool is_clear_path(const Point &start, const Point &end);
    bool is_point_valid(const Point &point);
    rrtNode* extend_tree(rrtNode* nearest, const Point &to);
    rrtNode* extend_rewire_tree(rrtNode* parent, const Point &sampled_point);
    std::function<void(const std::vector<rrtNode*>&)> visualization_callback;
    std::vector<Point> find_rrt_path();
    Point evaluate_bezier_de_casteljau(const std::vector<Point>& control_points, double t);
    std::vector<Point> smooth_rrt_path(const std::vector<Point>& rrt_path);
    RRT(rclcpp::Node* parent_node);
    ~RRT();
};
#endif