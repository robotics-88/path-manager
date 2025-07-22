/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef EXPLORER_H_
#define EXPLORER_H_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <explorer/decision_maker.h>
#include <explorer/open_space_finder.h>

namespace explorer
{
/**
 * @class Explorer
 * @brief Option-based exploration for a UAV using Mavlink/ArduPilot
 */
class Explorer
{
public:
  explicit Explorer(rclcpp::Node *node);
  ~Explorer(){};

  void initialize();
  void start();
  void stop();
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::vector<std::string> splitString(const std::string &s, char delim);

  geometry_msgs::msg::Pose makeNewGoal(const geometry_msgs::msg::Pose &position, double min_altitude, double max_altitude);

private:

  const std::shared_ptr<rclcpp::Node> node_;

  void visualizeOptions(const std::vector<decision_maker::NavOption>& options);
  void visualizeOptions();

  void visualizeGoal(decision_maker::NavOption& option);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  bool goalVisited(const geometry_msgs::msg::Point& goal);

  std::shared_ptr<decision_maker::DecisionMaker> decision_maker_;
  std::shared_ptr<open_space_finder::OpenSpaceFinder> open_space_finder_;

  // Frames
  std::string map_frame_;
  std::string slam_pose_topic_;

  float map_resolution_;
  float max_altitude_;
  float min_altitude_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_goal_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr restricted_option_publisher;

  std::vector<geometry_msgs::msg::Point> option_blacklist_;
  std::vector<geometry_msgs::msg::Point> option_visited_;
  geometry_msgs::msg::Point prev_goal_;
  size_t last_markers_count_;
  double default_alt_;

  bool visualize_;
  double min_option_size_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped current_pose_;
};
}

#endif
