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
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
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
    
    Cuboid(Point min, Point max) : min(min), max(max) {}
    
    //check if a point is inside the cuboid
    bool point_intersect(const Point &point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    //check if a line intersects the cuboid
    bool line_intersect(const Point &start, const Point &end, double safety_margin = 0.5) const {
        //add safety margin to point
        Point min_with_margin(min.x - safety_margin, min.y - safety_margin, min.z - safety_margin);
        Point max_with_margin(max.x + safety_margin, max.y + safety_margin, max.z + safety_margin);
        
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dz = end.z - start.z;
        
        const double epsilon = 1e-10;
        
        double tx1 = (dx == 0) ? -std::numeric_limits<double>::infinity() : (min_with_margin.x - start.x) / dx;
        double tx2 = (dx == 0) ? std::numeric_limits<double>::infinity() : (max_with_margin.x - start.x) / dx;
        double ty1 = (dy == 0) ? -std::numeric_limits<double>::infinity() : (min_with_margin.y - start.y) / dy;
        double ty2 = (dy == 0) ? std::numeric_limits<double>::infinity() : (max_with_margin.y - start.y) / dy;
        double tz1 = (dz == 0) ? -std::numeric_limits<double>::infinity() : (min_with_margin.z - start.z) / dz;
        double tz2 = (dz == 0) ? std::numeric_limits<double>::infinity() : (max_with_margin.z - start.z) / dz;
        
        if (tx1 > tx2) std::swap(tx1, tx2);
        if (ty1 > ty2) std::swap(ty1, ty2);
        if (tz1 > tz2) std::swap(tz1, tz2);
        
        double tmin = std::max(std::max(tx1, ty1), tz1);
        double tmax = std::min(std::min(tx2, ty2), tz2);
        
        return tmax >= tmin && tmax >= 0 && tmin <= 1;
    }
};

class RRT : public rclcpp::Node {
public:
    Point min_bound, max_bound;
    std::vector<Cuboid> obstacles {};
    rrtNode* root {};
    std::vector<rrtNode*> nodes {};
    double step_size; //dist between a new node and the nearest node in the tree
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
    Point bezier_point(const Point &p0, const Point &p1, const Point &p2, const Point &p3, double t);
    Point random_control_point(const Point &anchor, const Point &guide, double min_ratio, double max_ratio);
    std::vector<Point> bezier_path(const std::vector<Point> &path, int points_per_segment, int window_size, int max_attempts);
    std::function<void(const std::vector<rrtNode*>&)> visualization_callback;
    std::vector<Point> find_rrt_path(const Point &goal, int max_it);
    RRT(const Point &start, const Point &min_bound, const Point &max_bound, double step_size = 0.75);
    ~RRT();
};
#endif