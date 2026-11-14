#include "kait3_teleop/ds4_teleop_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kait3_teleop::DualShock4TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
