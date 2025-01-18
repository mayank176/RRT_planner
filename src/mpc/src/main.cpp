#include "sim.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}