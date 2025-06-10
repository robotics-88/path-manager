/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "path_manager/path_manager.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto pm_node = std::make_shared<path_manager::PathManager>();
  pm_node->initialize();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pm_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
