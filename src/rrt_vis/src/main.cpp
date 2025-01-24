#include "rrt_vis.h"
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}