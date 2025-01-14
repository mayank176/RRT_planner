#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <chrono>

#include "../../rrt/src/main.cpp"

class path_visualizer : public rclcpp::Node {

    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        std::unique_ptr<RRT> rrt_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<Point> path_;


        visualization_msgs::msg::Marker create_tree_marker() {

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

        visualization_msgs::msg::MarkerArray create_obstacle_marker() {
                
            visualization_msgs::msg::MarkerArray markers;
            int id = 1;

            for (const auto &obstacle : rrt_->obstacles) {
    
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "obstacles";
                marker.id = id++;
    
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
    
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;

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

        visualization_msgs::msg::Marker create_path_marker() {
            
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

        void visualize_and_plan() {

            visualization_msgs::msg::MarkerArray marker_array;

            auto tree_marker = create_tree_marker();
            auto obstacle_markers = create_obstacle_marker();

            marker_array.markers.push_back(tree_marker);
            marker_array.markers.insert(marker_array.markers.end(), obstacle_markers.markers.begin(), obstacle_markers.markers.end());

            if (!path_.empty()){
                auto path_marker = create_path_marker();
                marker_array.markers.push_back(path_marker);
            } else {
                Point goal(10.0, 10.0, 10.0);
                path_ = rrt_->find_rrt_path(goal);
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

    public:
        //Constructor
        path_visualizer() : Node("path_visualizer") {

            marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

            Point start(-9.0, -9.0, -9.0);
            Point goal(10.0, 10.0, 10.0);
            Point min_bound(-10.0, -10.0, -10.0);
            Point max_bound(11.0, 11.0, 11.0);

            rrt_ = std::make_unique<RRT>(start, min_bound, max_bound);

            Cuboid obstacle1(Point(-10.0, -10.0, -10.0), Point(11.0, 11.0, -9.75));
            rrt_->add_obstacle(obstacle1);

            Cuboid obstacle2(Point(-8.0, -8.0, -8.0), Point(-4.0, -4.0, 0.0));
            rrt_->add_obstacle(obstacle2);

            Cuboid obstacle3(Point(0.0, 0.0, 0.0), Point(8.0, 8.0, 8.0));
            rrt_->add_obstacle(obstacle3);
            
            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(
                500ms, 
                std::bind(&path_visualizer::visualize_and_plan, this));

            RCLCPP_INFO(this->get_logger(), "Path visualizer node has been started.");
        }

};

int main() {
    
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<path_visualizer>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}