/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "path_manager/path_manager.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_manager::PathManager>());
    rclcpp::shutdown();
    return 0;
}
