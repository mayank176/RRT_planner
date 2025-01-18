#include "rrt_vis.h"
int main() {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<path_visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}