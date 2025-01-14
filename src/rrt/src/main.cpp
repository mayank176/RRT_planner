#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Point{
    double x, y, z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}

    double distance(const Point &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + 
               std::pow(y - other.y, 2) + 
               std::pow(z - other.z, 2));
    }
};

struct rrtNode {
    Point position;
    rrtNode* parent;
    std::vector<rrtNode*> children;

    rrtNode(Point position, rrtNode* parent) : position(position), parent(parent) {}
};

//cuboid obstacles
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
    //Workspace bounds
    Point min_bound, max_bound;

    //Obstacles
    std::vector<Cuboid> obstacles {};

    //Tree
    rrtNode* root {};
    std::vector<rrtNode*> nodes {};
    //this is the maximum distance between a new node and the nearest node in the tree
    double step_size;

    //Random number generator
    std::mt19937 rng {};
    // std::random_device rd {}; 
    std::uniform_real_distribution<double> dist_x {};
    std::uniform_real_distribution<double> dist_y {};
    std::uniform_real_distribution<double> dist_z {};

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    //Constructor
    RRT(const Point &start, const Point &min_bound, const Point &max_bound, double step_size = 0.75)
    : Node("rrt_planner"), min_bound(min_bound), max_bound(max_bound), rng(std::random_device()()), step_size(step_size) {
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("rrt_path", 10);
        //initialize the tree with the start node
        std::cout << "RRT constructor" << std::endl;
        root = new rrtNode(start, nullptr); //nullptr means no parent
        nodes.push_back(root); 
        dist_x = std::uniform_real_distribution<double>(min_bound.x, max_bound.x); 
        dist_y = std::uniform_real_distribution<double>(min_bound.y, max_bound.y);
        dist_z = std::uniform_real_distribution<double>(min_bound.z, max_bound.z);
    } 

    //Destructor
    ~RRT() {
        for(rrtNode* node : nodes) {
            delete node;
        }
    }

    //setup the obstacles
    void add_obstacle(const Cuboid &obstacle) {
        obstacles.push_back(obstacle);
        std::cout << "Obstacle added!" << std::endl;
    }

    //Randomly sample a point in the workspace
    Point random_point() {
        return Point(dist_x(rng), dist_y(rng), dist_z(rng));
    }

    //Find the nearest node in the tree to a given point
    rrtNode* nearest_node(const Point &point) {
        rrtNode* nearest = nullptr;
        double min_distance = std::numeric_limits<double>::infinity();

        for (rrtNode* node : nodes) {
            double distance = node->position.distance(point); 
            //position.distance(point) means the distance between the position of the node and the given point
            if (distance < min_distance) {
                nearest = node;
                min_distance = distance;
            }
        }
        return nearest;
    }

    //Check if path is collision free
    bool is_clear_path(const Point &start, const Point &end){

        for (const Cuboid &obstacle : obstacles) {
            if (obstacle.line_intersect(start, end)) {
                return false;
            }
        }
        return true;
    }

    bool is_point_valid(const Point &point) {
        for (const auto &obstacle : obstacles) {
            if (obstacle.point_intersect(point)) {
                return false;
            }
        }
        return true;
    }

    //Extend the tree towards a point
    rrtNode* extend_tree(rrtNode* nearest, const Point &to){

        if (!is_clear_path(nearest->position, to)) {
            return nullptr;
        }
        double distance = nearest->position.distance(to);
        if (distance < step_size) {
            if (!is_point_valid(to)) {
                return nullptr;
            }
            rrtNode* new_node = new rrtNode(to, nearest);
            nearest->children.push_back(new_node);
            nodes.push_back(new_node);
            return new_node;
        } else {
            double dx = to.x - nearest->position.x;
            double dy = to.y - nearest->position.y;
            double dz = to.z - nearest->position.z;

            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            dx = dx / dist * step_size;
            dy = dy / dist * step_size;
            dz = dz / dist * step_size;

            Point new_position(nearest->position.x + dx, nearest->position.y + dy, nearest->position.z + dz);
            
            if (!is_point_valid(new_position)) {
                return nullptr;
            }

            if (is_clear_path(nearest->position, new_position)) {
                
                rrtNode* new_node = new rrtNode(new_position, nearest);
                nearest->children.push_back(new_node);
                nodes.push_back(new_node);
                return new_node;
            }
        }
        return nullptr;
    }

    //compute values of cubic bezier curve
    //B(t) = (1-t)^3 * P0 + 3(1-t)^2 * t * P1 + 3(1-t) * t^2 * P2 + t^3 * P3
    Point bezier_point(const Point &p0, const Point &p1, const Point &p2, const Point &p3, double t){
        return Point(
            (1-t)*(1-t)*(1-t)*p0.x + 3*(1-t)*(1-t)*t*p1.x + 3*(1-t)*t*t*p2.x + t*t*t*p3.x,
            (1-t)*(1-t)*(1-t)*p0.y + 3*(1-t)*(1-t)*t*p1.y + 3*(1-t)*t*t*p2.y + t*t*t*p3.y,
            (1-t)*(1-t)*(1-t)*p0.z + 3*(1-t)*(1-t)*t*p1.z + 3*(1-t)*t*t*p2.z + t*t*t*p3.z
        );
    }

    Point random_control_point(const Point &anchor, const Point &guide, double min_ratio = 0.1, double max_ratio = 0.9) {
       
        double ratio = min_ratio + (max_ratio - min_ratio) * (static_cast<double>(rand()) / RAND_MAX);
        
        return Point(
            ratio * anchor.x + (1 - ratio) * guide.x,
            ratio * anchor.y + (1 - ratio) * guide.y,
            ratio * anchor.z + (1 - ratio) * guide.z
        );
    }

    std::vector<Point> bezier_path(const std::vector<Point> &path, int points_per_segment = 50, int window_size = 5, int max_attempts = 100) {
        if (path.size() < window_size) return path;
        
        std::vector<Point> smoothed_path;
        smoothed_path.push_back(path[0]); 
        
        for (size_t i = 0; i < path.size() - 1; i += window_size-1) {
            // Get window of points for this segment
            std::vector<Point> window;
            for (size_t j = 0; j < window_size && (i + j) < path.size(); j++) {
                window.push_back(path[i + j]);
            }
            
            bool found_valid_curve = false;
            std::vector<Point> best_segment;
            
            // Try multiple times to find a valid bezier curve
            for (int attempt = 0; attempt < max_attempts && !found_valid_curve; attempt++) {
                std::vector<Point> current_segment;
                current_segment.push_back(smoothed_path.back());
                
                Point p0 = window[0];
                Point p3 = window[window.size() - 1];
                
                Point p1 = random_control_point(p0, window[1], 0.6, 0.9);  // Closer to p0
                Point p2 = random_control_point(p3, window[window.size() - 2], 0.6, 0.9);  // Closer to p3
                
                Point prev_point = current_segment.back();
                bool segment_valid = true;
                
                for (int j = 1; j <= points_per_segment; j++) {
                    double t = static_cast<double>(j) / points_per_segment;
                    Point bezier_pt = bezier_point(p0, p1, p2, p3, t);
                    
                    // Check if the new point and the path to it are safe
                    bool is_safe = true;
                    
                    // point collision
                    for (const auto& obstacle : obstacles) {
                        if (obstacle.point_intersect(bezier_pt)) {
                            is_safe = false;
                            break;
                        }
                    }
                    
                    // Check path collision
                    if (is_safe) {
                        const int subsegments = 10;
                        for (int k = 1; k <= subsegments; k++) {
                            double s = static_cast<double>(k) / subsegments;
                            Point intermediate(
                                prev_point.x + s * (bezier_pt.x - prev_point.x),
                                prev_point.y + s * (bezier_pt.y - prev_point.y),
                                prev_point.z + s * (bezier_pt.z - prev_point.z)
                            );
                            
                            if (!is_clear_path(prev_point, intermediate)) {
                                is_safe = false;
                                break;
                            }
                        }
                    }
                    
                    if (!is_safe) {
                        segment_valid = false;
                        break;
                    }
                    
                    current_segment.push_back(bezier_pt);
                    prev_point = bezier_pt;
                }
                
                if (segment_valid) {
                    found_valid_curve = true;
                    best_segment = current_segment;
                }
            }
            
            if (found_valid_curve) {
                smoothed_path.insert(smoothed_path.end(), 
                                best_segment.begin() + 1, 
                                best_segment.end());
            } else {
                throw std::runtime_error("Could not find valid bezier path for window");
            }
        }
        return smoothed_path;
    }

    //Find a path from the start to the goal
    std::vector<Point> find_rrt_path(const Point &goal, int max_it = 10000) {

        for (int i = 0; i < max_it; i++){
            
            // std::cout << "Iteration: " << i << std::endl;
            //Randomly sample a point in the workspace (Vanilla RRT)
            Point random = random_point();

            //Find the nearest node in the tree to the random point
            rrtNode* nearest = nearest_node(random);

            //Extend the tree towards the random point
            rrtNode* new_node = extend_tree(nearest, random);

            //Check if the new node is close to the goal
            if (new_node && new_node->position.distance(goal) < step_size) {
                if (!is_point_valid(goal)) { 
                    continue;
                }
                if(is_clear_path(new_node->position, goal)){
                    std::cout << "Goal found!" << std::endl;
                    std::vector<Point> path;
                    rrtNode* current = new_node;

                    bool path_valid = true;
                    while (current != nullptr && current->parent != nullptr) {
                        if (!is_clear_path(current->position, current->parent->position)) {
                            path_valid = false;
                            break;
                        }
                        path.push_back(current->position);
                        current = current->parent;
                    }
                    
                    if (!path_valid) {
                        continue;
                    }
                    
                    if (current != nullptr) {
                        path.push_back(current->position);
                    }

                    std::reverse(path.begin(), path.end());
                    path.push_back(goal);
                    auto path_msg = nav_msgs::msg::Path();
                    path_msg.header.frame_id = "map";
                    path_msg.header.stamp = this->now();

                    for (const auto& point : path) {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path_msg.header;
                        pose.pose.position.x = point.x;
                        pose.pose.position.y = point.y;
                        pose.pose.position.z = point.z;
                        pose.pose.orientation.w = 1.0;  // Default orientation
                        path_msg.poses.push_back(pose);
                    }
                    path_pub_ -> publish(path_msg);
                    RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path.size());
                    return (path);
                }
            }
        }
        return std::vector<Point>();
    }
};

// int main() {

//     Point start(0.0, 0.0, 0.0);
//     Point goal(10.0, 10.0, 10.0);
//     Point min_bound(-10.0, -10.0, -10.0);
//     Point max_bound(11.0, 11.0, 11.0);
//     std::cout << "Start: " << start.x << " " << start.y << " " << start.z << std::endl;
//     std::cout << "About to create RRT " << std::endl;

//     RRT rrt(start, min_bound, max_bound);
//     std::cout << "RRT created!" << std::endl;

//     Cuboid obstacle1(Point(-5.0, -5.0, -5.0), Point(5.0, 5.0, 5.0));
//     // rrt.add_obstacle(obstacle1);

//     std::vector<Point> path = rrt.find_rrt_path(goal);

//     for (const Point &point : path) {
//         std::cout << point.x << " " << point.y << " " << point.z << std::endl;
//     }

//     return 0;
// }