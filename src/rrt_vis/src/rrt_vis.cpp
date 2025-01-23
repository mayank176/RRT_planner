#include "rrt_vis.h"

visualization_msgs::msg::Marker path_visualizer::create_tree_marker() {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();

    marker.ns = "rrt_tree";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;

    for (const auto &node : rrt_->nodes) {
        if (node->parent) {
            geometry_msgs::msg::Point p1,p2;

            p1.x = node->parent->position.x;
            p1.y = node->parent->position.y;
            p1.z = node->parent->position.z;

            p2.x = node->position.x;
            p2.y = node->position.y;
            p2.z = node->position.z;

            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }
    }
    return marker;
}

visualization_msgs::msg::MarkerArray path_visualizer::create_obstacle_marker() {
        
    visualization_msgs::msg::MarkerArray markers;
    int id = 1;
    bool is_first_obstacle = true;
    for (const auto &obstacle : rrt_->obstacles) {

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = id++;

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        if (is_first_obstacle) {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            is_first_obstacle = false;
        } else {
            marker.color.r = 0.4;
            marker.color.g = 0.4;
            marker.color.b = 0.45;
            marker.color.a = 1.0;
        }

        marker.pose.position.x = (obstacle.min.x + obstacle.max.x) / 2.0;
        marker.pose.position.y = (obstacle.min.y + obstacle.max.y) / 2.0;
        marker.pose.position.z = (obstacle.min.z + obstacle.max.z) / 2.0;

        marker.scale.x = obstacle.max.x - obstacle.min.x;
        marker.scale.y = obstacle.max.y - obstacle.min.y;
        marker.scale.z = obstacle.max.z - obstacle.min.z;

        markers.markers.push_back(marker);
    }
    return markers;
}

visualization_msgs::msg::Marker path_visualizer::create_path_marker() {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();

    marker.ns = "path";
    marker.id = 2;

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto &point : path_) {
        geometry_msgs::msg::Point p;

        p.x = point.x;
        p.y = point.y;
        p.z = point.z;

        marker.points.push_back(p);
    }
    return marker;
}

void path_visualizer::visualize_and_plan() {

    visualization_msgs::msg::MarkerArray marker_array;

    auto tree_marker = create_tree_marker();
    auto obstacle_markers = create_obstacle_marker();

    marker_array.markers.push_back(tree_marker);
    marker_array.markers.insert(marker_array.markers.end(), obstacle_markers.markers.begin(), obstacle_markers.markers.end());

    if (!path_.empty()){
        auto path_marker = create_path_marker();
        marker_array.markers.push_back(path_marker);
    } else if (!planning_in_progress_){
        planning_in_progress_ = true;
        
        // Set up callback for tree visualization
        rrt_->visualization_callback = [this](const std::vector<rrtNode*>& nodes) {
            visualization_msgs::msg::MarkerArray update_markers;
            
            // Create and publish tree marker
            auto tree_marker = create_tree_marker();
            update_markers.markers.push_back(tree_marker);
            
            // Add obstacle markers
            auto obstacle_markers = create_obstacle_marker();
            update_markers.markers.insert(  update_markers.markers.end(), 
                                            obstacle_markers.markers.begin(), 
                                            obstacle_markers.markers.end());
            
            marker_publisher_->publish(update_markers);
            
            // Add small delay to make visualization visible
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        };

        path_ = rrt_->find_rrt_path();
        planning_in_progress_ = false;
        if (!path_.empty()){
            //print path_
            for (const auto &point : path_) {
                RCLCPP_INFO(this->get_logger(), "Path: %f %f %f", point.x, point.y, point.z);
            }
            RCLCPP_INFO(this->get_logger(), "Path found!");
        }
    }

    marker_publisher_->publish(marker_array);
}


path_visualizer::path_visualizer() : Node("path_visualizer") {

    declare_parameter("min_bound_x", -10.0);
    declare_parameter("min_bound_y", -10.0);
    declare_parameter("min_bound_z", -10.0);
    declare_parameter("max_bound_x", 11.0);
    declare_parameter("max_bound_y", 11.0);
    declare_parameter("max_bound_z", 11.0);
    declare_parameter("step_size", 0.75);
    declare_parameter("safety_margin", 0.7);
    declare_parameter("max_iterations", 10000);
    
    declare_parameter("start_x", -9.0);
    declare_parameter("start_y", -9.0);
    declare_parameter("start_z", -9.0);
    declare_parameter("goal_x", 10.0);
    declare_parameter("goal_y", 10.0);
    declare_parameter("goal_z", 10.0);
    
    declare_parameter("environment_num", 2);
    
    int environment_num = this ->get_parameter("environment_num").as_int();
    
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    
    rrt_ = std::make_unique<RRT>(this);

    Cuboid obstacle1(Point(-10.0, -10.0, -10.0), Point(11.0, 11.0, -9.75));
    rrt_->add_obstacle(obstacle1);

    if (environment_num == 1){
        Cuboid obstacle2(Point(-4.0, -10.0, -10.0), Point(-4.5, 4.0, 11.0));
        rrt_->add_obstacle(obstacle2);
        Cuboid obstacle3(Point(4.0, -4.0, -10.0), Point(4.5, 11.0, 11.0));
        rrt_->add_obstacle(obstacle3);
    }
    if (environment_num == 2){
        Cuboid obstacle2(Point(-4.0, -10.0, -10.0), Point(-4.5, 11.0, 4.0));
        rrt_->add_obstacle(obstacle2);
        Cuboid obstacle3(Point(-4.0, -10.0, 8.0 ), Point(-4.5, 11.0, 11.0));
        rrt_->add_obstacle(obstacle3);
        Cuboid obstacle4(Point(4.0, -10.0, -4.0), Point(4.5, 11.0, 11.0));
        rrt_->add_obstacle(obstacle4);
        Cuboid obstacle5(Point(4.0, -10.0, -10.0 ), Point(4.5, 11.0, -7.0));
        rrt_->add_obstacle(obstacle5);
    }
    if (environment_num == 3){
        Cuboid obstacle2(Point(-8.0, -8.0, -10.0), Point(-6.0, -6.0, 11.0));
        rrt_->add_obstacle(obstacle2);
        Cuboid obstacle3(Point(0.0, 0.0, -10.0), Point(-3.0, -3.0, 11.0));
        rrt_->add_obstacle(obstacle3);
        Cuboid obstacle4(Point(2.0, 0.0, -10.0), Point(4.0, 2.0, 11.0));//8
        rrt_->add_obstacle(obstacle4);
        Cuboid obstacle5(Point(5.0, 5.0, -10.0), Point(8.0, 8.0, 11.0));//8
        rrt_->add_obstacle(obstacle5);
        Cuboid obstacle6(Point(-9.0, 1.0, -10.0), Point(-6.0, -3.0, 11.0));
        rrt_->add_obstacle(obstacle6);
        Cuboid obstacle7(Point(9.0, -1.0, -10.0), Point(6.0, 3.0, 11.0));//8
        rrt_->add_obstacle(obstacle7);
        Cuboid obstacle8(Point(-1.0, -9.0, -10.0), Point(3.0, -6.0, 11.0));//8
        rrt_->add_obstacle(obstacle8);
        Cuboid obstacle9(Point(8.0, -9.0, -10.0), Point(6.0, -6.0, 11.0));//8
        rrt_->add_obstacle(obstacle9);
        Cuboid obstacle10(Point(-8.0, 9.0, -10.0), Point(-6.0, 6.0,11.0));//8
        rrt_->add_obstacle(obstacle10);
        Cuboid obstacle11(Point(-1.0, 7.0, -10.0), Point(1.0, 5.0, 11.0));
        rrt_->add_obstacle(obstacle11);
    }
    
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        10ms, 
        std::bind(&path_visualizer::visualize_and_plan, this));

    RCLCPP_INFO(this->get_logger(), "Path visualizer node has been started.");
}