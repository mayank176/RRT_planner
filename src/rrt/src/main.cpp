#include "rrt.h"

Point::Point(double x, double y, double z) : x(x), y(y), z(z) {}

Cuboid::Cuboid(Point min_, Point max_) : min(min_), max(max_) {
        // Ensure min is actually less than max in all dimensions
        Point true_min(
            std::min(min_.x, max_.x),
            std::min(min_.y, max_.y),
            std::min(min_.z, max_.z)
        );
        Point true_max(
            std::max(min_.x, max_.x),
            std::max(min_.y, max_.y),
            std::max(min_.z, max_.z)
        );
        min = true_min;
        max = true_max;
        }

bool Cuboid::point_intersect(const Point &point, double safety_margin) const {
    
    bool x_in = point.x >= (min.x - safety_margin) && point.x <= (max.x + safety_margin);
    bool y_in = point.y >= (min.y - safety_margin) && point.y <= (max.y + safety_margin);
    bool z_in = point.z >= (min.z - safety_margin) && point.z <= (max.z + safety_margin);
    return x_in && y_in && z_in;
}

bool Cuboid::line_intersect(const Point &start, const Point &end, double safety_margin) const {
    
    Point min_with_margin(min.x - safety_margin, min.y - safety_margin, min.z - safety_margin);
    Point max_with_margin(max.x + safety_margin, max.y + safety_margin, max.z + safety_margin);
    
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dz = end.z - start.z;
    double total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    const double step_size = 0.1;
    int num_steps = std::ceil(total_distance / step_size);
    
    double step_dx = dx / total_distance * step_size;
    double step_dy = dy / total_distance * step_size;
    double step_dz = dz / total_distance * step_size;

    for (int i = 0; i <= num_steps; i++) {
        Point sample_pos(
            start.x + i * step_dx,
            start.y + i * step_dy,
            start.z + i * step_dz
        );
        if (point_intersect(sample_pos, safety_margin)) {
            return true;  // Found a collision
        }
    }
    return false;
}
    

RRT::RRT(rclcpp::Node* parent_node) 
: Node("rrt_planner") {
    
    min_bound = Point(  parent_node->get_parameter("min_bound_x").as_double(),
                        parent_node->get_parameter("min_bound_y").as_double(),
                        parent_node->get_parameter("min_bound_z").as_double()
    );
    max_bound = Point(  parent_node->get_parameter("max_bound_x").as_double(),
                        parent_node->get_parameter("max_bound_y").as_double(),
                        parent_node->get_parameter("max_bound_z").as_double()
    );
    start = Point(  parent_node->get_parameter("start_x").as_double(),
                    parent_node->get_parameter("start_y").as_double(),
                    parent_node->get_parameter("start_z").as_double()
    );
    goal = Point(   parent_node->get_parameter("goal_x").as_double(),
                    parent_node->get_parameter("goal_y").as_double(),
                    parent_node->get_parameter("goal_z").as_double()
    );

    step_size = parent_node->get_parameter("step_size").as_double();
    safety_margin = parent_node->get_parameter("safety_margin").as_double();        
    max_iterations = parent_node->get_parameter("max_iterations").as_int();
    env_num = parent_node->get_parameter("environment_num").as_int();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("rrt_path", 10);
    //initialize the tree with the start node
    std::cout << "RRT constructor" << std::endl;
    root = new rrtNode(start, nullptr); //nullptr means no parent
    nodes.push_back(root); 
    dist_x = std::uniform_real_distribution<double>(min_bound.x, max_bound.x); 
    dist_y = std::uniform_real_distribution<double>(min_bound.y, max_bound.y);
    dist_z = std::uniform_real_distribution<double>(min_bound.z, max_bound.z);
} 

RRT::~RRT() {
    for(rrtNode* node : nodes) {
        delete node;
    }
}

void RRT::add_obstacle(const Cuboid &obstacle) {
    obstacles.push_back(obstacle);
    RCLCPP_INFO(this->get_logger(), "obstacle added");
}

Point RRT::random_point() {
    return Point(dist_x(rng), dist_y(rng), dist_z(rng));
}

//Find the nearest node in the tree to a given point
rrtNode* RRT::nearest_node(const Point &point) {
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
bool RRT::is_clear_path(const Point &start, const Point &end){

    for (const Cuboid &obstacle : obstacles) {
        if (obstacle.line_intersect(start, end, safety_margin)) {
            return false;
        }
    }
    return true;
}

bool RRT::is_point_valid(const Point &point) {
    for (const auto &obstacle : obstacles) {
        if (obstacle.point_intersect(point, safety_margin)) {
            return false;
        }
    }
    return true;
}

//Extend the tree towards a point
rrtNode* RRT::extend_tree(rrtNode* nearest, const Point &to){

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

//Evaluate a Bezier curve using De Casteljau's algorithm
Point RRT::evaluate_bezier_de_casteljau(const std::vector<Point>& control_points, double t) {
    if (control_points.size() == 1) return control_points[0];
    
    std::vector<Point> points = control_points;
    
    // Apply de Casteljau's algorithm
    for (size_t r = 1; r < control_points.size(); ++r) {
        for (size_t i = 0; i < control_points.size() - r; ++i) {
            points[i] = Point(
                (1.0 - t) * points[i].x + t * points[i + 1].x,
                (1.0 - t) * points[i].y + t * points[i + 1].y,
                (1.0 - t) * points[i].z + t * points[i + 1].z
            );
        }
    }
    return points[0];
}

std::vector<Point> RRT::smooth_rrt_path(const std::vector<Point>& rrt_path) {
    std::vector<Point> smoothed_path;
    
    size_t i = 0;
    while(i < rrt_path.size() - 1) {  
        
        size_t points_remaining = rrt_path.size() - i;
        size_t num_control_points = std::min(points_remaining, size_t(50));  
        std::vector<Point> control_points;

        for(size_t j = 0; j < num_control_points; j++) {
            control_points.push_back(rrt_path[i + j]);
        }

        size_t points_to_generate = (num_control_points > 1) ? num_control_points - 1 : 1;
        double t_step = 1.0 / (points_to_generate);

        for (size_t j = 0; j < points_to_generate; j++) {
            double t = j * t_step;
            Point p = evaluate_bezier_de_casteljau(control_points, t);
            if(is_point_valid(p)) {
                smoothed_path.push_back(p);
            } 
        }
        i += (num_control_points > 1) ? num_control_points - 1 : 1;
    }
    return smoothed_path;
}

//Find a path from the start to the goal
std::vector<Point> RRT::find_rrt_path() {

    for (int i = 0; i < max_iterations; i++){
        
        // std::cout << "Iteration: " << i << std::endl;
        //Randomly sample a point in the workspace (Vanilla RRT)
        Point random = random_point();

        //Find the nearest node in the tree to the random point
        rrtNode* nearest = nearest_node(random);

        //Extend the tree towards the random point
        rrtNode* new_node = extend_tree(nearest, random);

        if (i % 50 == 0 && visualization_callback) {  // Adjust frequency as needed
            visualization_callback(nodes);
        }

        //Check if the new node is close to the goal
        if (new_node && new_node->position.distance(goal) < step_size) {
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
                //******Smooth path with Bezier curve:******/
                path = smooth_rrt_path(path);
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
                return path;
            }
        }
    }
    return std::vector<Point>();
}