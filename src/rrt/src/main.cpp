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

//RRT*
rrtNode* RRT::extend_rewire_tree(rrtNode* parent, const Point &sampled_point) {
    
    //Extend Tree like in vanilla RRT:
    rrtNode* new_node = extend_tree(parent, sampled_point);

    if (new_node == nullptr){ //extend_tree outputs nullptr when the sampled point is in an obstacle, then we skip this point
        return nullptr;
    } else {
        double n = nodes.size();
        double r = std::min(9.0 * std::pow((std::log(n) / n), 0.25), step_size);

        std::vector<rrtNode*> nearby_nodes; 
        for (rrtNode* node: nodes){ //check-out nodes in radius 'r' around 
            if (node->position.distance(new_node->position) < r && is_clear_path(node->position, new_node->position)){
                nearby_nodes.push_back(node);
            }
        }

        //Find a better parent for new_node
        rrtNode* best_parent = nullptr;

        double min_cost = 0;
        rrtNode* cur = new_node;
        while (cur->parent != nullptr) {
            min_cost += cur->position.distance(cur->parent->position);
            cur = cur->parent;
        }

        //Try to find better parent
        for (rrtNode* potential_parent: nearby_nodes) {
            //Calculate cost from start to potential_parent
            double cost_to_parent = 0;
            cur = potential_parent;
            while (cur->parent != nullptr) {
                cost_to_parent += cur->position.distance(cur->parent->position);
                cur = cur->parent;
            }
            //Add cost from potential_parent to new_node
            double total_cost = cost_to_parent + potential_parent->position.distance(new_node->position);
            
            if (total_cost < min_cost) {
                best_parent = potential_parent;
                min_cost = total_cost;
            }
        }

        // Rewire nearby nodes
        for (rrtNode* node: nearby_nodes) {
            if (node == best_parent) {
                continue;
            }
            
            //Calculate current cost to node
            double current_cost = 0;
            cur = node;
            while (cur->parent != nullptr) {
                current_cost += cur->position.distance(cur->parent->position);
                cur = cur->parent;
            }

            //Calculate potential cost through new_node
            double cost_to_new = 0;
            cur = new_node;
            while (cur->parent != nullptr) {
                cost_to_new += cur->position.distance(cur->parent->position);
                cur = cur->parent;
            }
            double potential_cost = cost_to_new + new_node->position.distance(node->position);

            if (potential_cost < current_cost) {
                node->parent = new_node;
            }
        }
        return new_node;
    }    
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
        size_t num_control_points = std::min(points_remaining, size_t(10));  
        std::vector<Point> control_points;

        for(size_t j = 0; j < num_control_points; j++) {
            control_points.push_back(rrt_path[i + j]);
        }

        size_t points_to_generate = (num_control_points > 1) ? num_control_points - 1 : 1;
        double t_step = 1.0 / (points_to_generate*2);

        for (size_t j = 0; j < points_to_generate*2; j++) {
            double t = j * t_step;
            Point p = evaluate_bezier_de_casteljau(control_points, t);
            if(is_point_valid(p)) {
                smoothed_path.push_back(p);
            } else {
                RCLCPP_INFO(this->get_logger(), "Point not valid");
            }
        }
        i += (num_control_points > 1) ? num_control_points - 1 : 1;
    }
    return smoothed_path;
}

Point RRT::sample_unit_ball() {
    double r = std::pow(dist_x(rng), 1.0/3.0);  // Cubic root for uniform distribution
    double theta = dist_x(rng) * 2 * M_PI;
    double phi = std::acos(2 * dist_x(rng) - 1);
    
    return Point(
        r * std::sin(phi) * std::cos(theta),
        r * std::sin(phi) * std::sin(theta),
        r * std::cos(phi)
    );
}

// Add rotation matrix calculation
Eigen::Matrix3d RRT::rotation_to_world_frame() {
    Point c_best = goal - start;  // Direction vector from start to goal
    c_best = c_best * (1.0 / c_best.distance(Point(0,0,0)));  // Normalize
    
    // Create rotation matrix using the direction vector
    Eigen::Vector3d z_world(0, 0, 1);
    Eigen::Vector3d c_vec(c_best.x, c_best.y, c_best.z);
    Eigen::Vector3d x_world = c_vec;
    Eigen::Vector3d y_world = z_world.cross(x_world);
    y_world.normalize();
    z_world = x_world.cross(y_world);
    
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_world;
    rotation.col(1) = y_world;
    rotation.col(2) = z_world;
    
    return rotation;
}

// Modify random point sampling for Informed RRT*
Point RRT::random_point_informed() {
    if (best_solution_node == nullptr) {
        return random_point();  // Use regular sampling if no solution found yet
    }
    
    double c_min = start.distance(goal);  // Minimum possible cost
    double c_max = best_solution_cost;    // Current best cost
    
    // Calculate ellipsoid parameters
    double c = c_max / 2.0;
    double a = c;
    double r = std::sqrt(c_max * c_max - c_min * c_min) / 2.0;
    
    // Sample from unit ball and transform to ellipsoid
    Point ball_point = sample_unit_ball();
    Eigen::Vector3d x_ball(ball_point.x, ball_point.y, ball_point.z);
    
    // Transform to ellipsoid
    Eigen::Vector3d x_ellipse = x_ball;
    x_ellipse(0) *= a;
    x_ellipse(1) *= r;
    x_ellipse(2) *= r;
    
    // Rotate to world frame
    Eigen::Matrix3d rotation = rotation_to_world_frame();
    Eigen::Vector3d x_world = rotation * x_ellipse;
    
    // Translate relative to start position
    Point center = Point(
        (start.x + goal.x) / 2.0,
        (start.y + goal.y) / 2.0,
        (start.z + goal.z) / 2.0
    );
    
    return Point(
        x_world(0) + center.x,
        x_world(1) + center.y,
        x_world(2) + center.z
    );
}

//Find a path from the start to the goal
std::vector<Point> RRT::find_rrt_path() {
    rrtNode* best_node = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < max_iterations; i++){
        // RCLCPP_INFO(this->get_logger(), "iteration: %d", i);
        
        //Randomly sample a point in the workspace (Vanilla RRT)
        Point random = random_point();
        // Point random = random_point_informed(); //informed version

        //Find the nearest node in the tree to the random point
        rrtNode* nearest = nearest_node(random);

        //Extend the tree towards the random point
        // rrtNode* new_node = extend_tree(nearest, random); //Vanilla RRT
        rrtNode* new_node = extend_rewire_tree(nearest, random); //RRT*

        if (i % 50 == 0 && visualization_callback) {  // Adjust frequency as needed
            visualization_callback(nodes);
        }
        // RCLCPP_INFO(this->get_logger(), "Step_size: %f", step_size);

        if (new_node && new_node->position.distance(goal) < step_size) {
            if(is_clear_path(new_node->position, goal)) {
                double path_cost = 0;
                rrtNode* current = new_node;
                bool path_valid = true;

                // Calculate cost of path
                path_cost += goal.distance(current->position);
                while (current != nullptr && current->parent != nullptr) {
                    if (!is_clear_path(current->position, current->parent->position)) {
                        path_valid = false;
                        break;
                    }
                    path_cost += current->position.distance(current->parent->position);
                    current = current->parent;
                }
                
                if (!path_valid) continue;
                

                 RCLCPP_INFO(this->get_logger(), 
            "Found potential path with cost: %.6f (current best: %.6f) at iteration %d", 
            path_cost, best_cost, i);
                

                // Only save the node and cost if better
                if (path_cost < best_cost) {
                    best_cost = path_cost;
                    best_node = new_node;
                }
            }
        }
    }
    
    // // After all iterations, construct the path from the best node
    std::vector<Point> best_path;
    if (best_node != nullptr) {
        rrtNode* current = best_node;
        while (current != nullptr && current->parent != nullptr) {
            best_path.push_back(current->position);
            current = current->parent;
        }
        if (current != nullptr) {
            best_path.push_back(current->position);
        }
        std::reverse(best_path.begin(), best_path.end());
        best_path.push_back(goal);
        
        // Smooth and publish only the final path
        best_path = smooth_rrt_path(best_path);
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
    
        for (const auto& point : best_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = point.z;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published final best path with cost: %f", best_cost);
    }
    
    return best_path;
}















//         //Check if the new node is close to the goal
//         if (new_node && new_node->position.distance(goal) < step_size) {
//             if(is_clear_path(new_node->position, goal)){
//                 RCLCPP_INFO(this->get_logger(), "Goal Found!");
//                 std::vector<Point> path;
//                 rrtNode* current = new_node;

//                 bool path_valid = true;
//                 while (current != nullptr && current->parent != nullptr) {
//                     if (!is_clear_path(current->position, current->parent->position)) {
//                         path_valid = false;
//                         RCLCPP_INFO(this->get_logger(), "Path not valid!");
//                         break;
//                     }
//                     path.push_back(current->position);
//                     current = current->parent;
//                 }

//                 if (!path_valid) {
//                     continue;
//                 }

//                 if (current != nullptr) {
//                     path.push_back(current->position);
//                 }

//                 std::reverse(path.begin(), path.end());
//                 path.push_back(goal);
//                 //******Smooth path with Bezier curve:******/
//                 path = smooth_rrt_path(path);
//                 auto path_msg = nav_msgs::msg::Path();
//                 path_msg.header.frame_id = "map";
//                 path_msg.header.stamp = this->now();

//                 for (const auto& point : path) {
//                     geometry_msgs::msg::PoseStamped pose;
//                     pose.header = path_msg.header;
//                     pose.pose.position.x = point.x;
//                     pose.pose.position.y = point.y;
//                     pose.pose.position.z = point.z;
//                     pose.pose.orientation.w = 1.0;  // Default orientation
//                     path_msg.poses.push_back(pose);
//                 }
//                 path_pub_ -> publish(path_msg);
//                 RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path.size());
//                 return path;
//             }
//         }
//     }
//     return std::vector<Point>();
// }


//**************************************/
