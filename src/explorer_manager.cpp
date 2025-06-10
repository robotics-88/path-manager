/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include <explorer/explorer_manager.h>

#include <thread>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace explorer
{
Explorer::Explorer(rclcpp::Node *node)
  : node_(node)
  , decision_maker_(nullptr)
  , open_space_finder_(nullptr)
  , map_frame_("map")
  , slam_pose_topic_("/mavros/vision_pose/pose")
  , map_resolution_(0.5)
  , last_markers_count_(0)
  , default_alt_(2.0)
  , visualize_(true)
{
}

void Explorer::initialize() {
    decision_maker_ = std::make_shared<decision_maker::DecisionMaker>();
    open_space_finder_ = std::make_shared<open_space_finder::OpenSpaceFinder>();

  node_->declare_parameter("map_frame", map_frame_);
  node_->declare_parameter("slam_pose_topic", slam_pose_topic_);
  node_->declare_parameter("visualize", visualize_);
  std::string goal_topic = "/goal";
  node_->declare_parameter("goal_topic", goal_topic);
  node_->declare_parameter("map_resolution", map_resolution_);
  std::string crit_string, wt_string;
  node_->declare_parameter("criteria_array", crit_string);
  node_->declare_parameter("weights_array", wt_string);

  node_->get_parameter("map_frame", map_frame_);
  node_->get_parameter("slam_pose_topic", slam_pose_topic_);
  node_->get_parameter("visualize", visualize_);
  node_->get_parameter("goal_topic", goal_topic);
  node_->get_parameter("map_resolution", map_resolution_);
  // already declared by path manager
  node_->get_parameter("default_alt", default_alt_);

  // Criteria and weights need parsing
  node_->get_parameter("criteria_array", crit_string);
  std::vector<std::string> criteria = splitString(crit_string, ',');
  node_->get_parameter("weights_array", wt_string);
  std::vector<std::string> wt_vec_string = splitString(wt_string, ',');
  std::vector<double> weights;
  for (int ii = 0; ii < wt_vec_string.size(); ii++) {
    weights.push_back(std::stod(wt_vec_string.at(ii)));
  }

  // Open Space Finder parameters
  node_->declare_parameter<float>("search_radius", 5.0);
  node_->declare_parameter<float>("voxel_size", 0.5);
  node_->declare_parameter<float>("min_distance_threshold", 1.0);
  node_->declare_parameter<int>("max_open_centers", 10);
  node_->declare_parameter<float>("z_limit", 2.0);
  node_->declare_parameter<std::string>("pointcloud_topic", "/cloud_aggregated");

  float search_radius_, 
        voxel_size_, 
        min_distance_threshold_, 
        z_limit_;
  int max_open_centers_;
  std::string pointcloud_topic;
  node_->get_parameter("search_radius", search_radius_);
  node_->get_parameter("voxel_size", voxel_size_);
  node_->get_parameter("min_distance_threshold", min_distance_threshold_);
  node_->get_parameter("max_open_centers", max_open_centers_);
  node_->get_parameter("z_limit", z_limit_);
  node_->get_parameter("pointcloud_topic", pointcloud_topic);
  open_space_finder_->initialize(
      search_radius_, voxel_size_, min_distance_threshold_,
      max_open_centers_, z_limit_);

  // Init prev goal for decision_maker
  prev_goal_.x = 0;
  prev_goal_.y = 0;
  prev_goal_.z = 0;
  option_blacklist_.push_back(prev_goal_); // Origin is never a good target, but sometimes gets stuck
  decision_maker_->init( map_resolution_, criteria, weights);

  if (visualize_) {
    marker_array_publisher_ =
                    node_->create_publisher<visualization_msgs::msg::MarkerArray>("options", 10);
    marker_goal_publisher_ =
                    node_->create_publisher<visualization_msgs::msg::Marker>("option_goal_marker", 10);
    restricted_option_publisher =
                    node_->create_publisher<visualization_msgs::msg::MarkerArray>("restricted_options", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("open_regions", 10);
  }

  // TODO remove subs from here, just path manager pass them to explorer
  pose_sub_ =       node_->create_subscription<geometry_msgs::msg::PoseStamped>(slam_pose_topic_, 10, std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10, std::bind(&Explorer::cloudCallback, this, std::placeholders::_1));
    
}

std::vector<std::string> Explorer::splitString(const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}

void Explorer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = *msg;
  open_space_finder_->setPose(msg);
}

void Explorer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  open_space_finder_->setCloud(msg);
}

void Explorer::visualizeOptions()
{
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 0.5;
  std_msgs::msg::ColorRGBA yellow;
  yellow.r = 1.0;
  yellow.g = 1.0;
  yellow.b = 0;
  yellow.a = 0.5;

  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = map_frame_;
  m.header.stamp = node_->get_clock()->now();
  m.ns = "options";
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = rclcpp::Duration(0.0, 0.0);
  m.frame_locked = true;

  m.action = visualization_msgs::msg::Marker::ADD;
  int id = 0;
  for (auto& option : option_blacklist_) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = id;
    m.pose.position = option;
    m.pose.orientation.w = 1.0;
    double scale = 1.5;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = red;
    markers.push_back(m);
    ++id;
  }

  for (auto& option : option_visited_) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = id;
    m.pose.position = option;
    m.pose.orientation.w = 1.0;
    double scale = 3.0;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = yellow;
    markers.push_back(m);
    ++id;
  }

  restricted_option_publisher->publish(markers_msg);
}

void Explorer::visualizeOptions(
    const std::vector<decision_maker::NavOption>& options)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::msg::ColorRGBA yellow;
  yellow.r = 1.0;
  yellow.g = 1.0;
  yellow.b = 0;
  yellow.a = 1.0;

  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = map_frame_;
  m.header.stamp = node_->get_clock()->now();
  m.ns = "options";
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = rclcpp::Duration(0.0, 0.0);
  m.frame_locked = true;

  // weighted options are always sorted
  double min_cost = options.empty() ? 0. : options.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& option : options) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = option.location;
    m.pose.orientation.w = 1.0;
    // TODO harcode or scale option according to its cost (costlier options will be smaller)?
    double scale = 0.5;// std::max(std::abs(min_cost * 3.0 / option.cost), 3.5);
    if (isinf(scale)) scale = 10.0;
    if (isnan(scale)) scale = 1.0;
    if (scale < 1.0) scale = 1.0;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    if (goalOnBlacklist(option.location)) {
      m.color = red;
    }
    else if (goalVisited(option.location)) {
      m.color = yellow;
    } else {
      m.color = green;
    }
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
  visualizeOptions();
}

geometry_msgs::msg::Pose Explorer::makeNewGoal(const geometry_msgs::msg::Pose &position, double min_altitude, double max_altitude) {
  std::vector<decision_maker::NavOption> options = open_space_finder_->searchFrom(position, min_altitude, max_altitude);
  if (visualize_) {
    visualization_msgs::msg::Marker marker = open_space_finder_->getMarkers();
    marker.header = current_pose_.header; // Improve later, but pretty close
    marker_pub_->publish(marker);
  }

  if (options.empty()) {
    // TODO add handling in path manager for this case, means there is no open space found near the goal, shouldnt fly there. do recovery behavior.
    RCLCPP_INFO(node_->get_logger(), "Stopping due to exploration complete, no options found in region.");
    // geometry_msgs::msg::Pose result;
    // result.position.x = 0; // return a zeroed pose if no options found  
    // result.position.y = 0; // indicates failure to find a goal
    // result.position.z = 0; // indicates failure to find a goal
    return position; // Not what we want but leaving for testing
  }

  // Fill in option cost
  decision_maker_->evaluate(options, prev_goal_);

  // publish options as visualization markers
  if (visualize_) {
    visualizeOptions(options);
  }

  // find non blacklisted option-original version.
  auto option =
      std::find_if_not(options.begin(), options.end(),
                       [this](const decision_maker::NavOption& f) {
                         return (goalOnBlacklist(f.location) || goalVisited(f.location));
                       });

  //check if no options returned
  if (option == options.end()) {
    RCLCPP_INFO(node_->get_logger(), "Stopping due to exploration complete, no non-blacklisted options found in region.");
    geometry_msgs::msg::Pose result;
    result.position.x = 0; // return a zeroed pose if no options found  
    result.position.y = 0; // node_ indicates failure to find a goal
    result.position.z = 0; // node_ indicates failure to find a goal
    return result;
  }
  geometry_msgs::msg::Point target_position = option->location;
  prev_goal_ = target_position;

  decision_maker::NavOption & opt = *option;
  Explorer::visualizeGoal(opt);
  RCLCPP_INFO(node_->get_logger(), "Exploration found new goal.");

  geometry_msgs::msg::Pose target_pos_msg;
  target_pos_msg.position = target_position;
  return target_pos_msg;
}

bool Explorer::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerance = 1.5;

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& option_goal : option_blacklist_) {
    double x_diff = fabs(goal.x - option_goal.x);
    double y_diff = fabs(goal.y - option_goal.y);

    if (x_diff < tolerance * map_resolution_ &&
        y_diff < tolerance * map_resolution_)
      return true;
  }
  return false;
}

bool Explorer::goalVisited(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerance = 1.5;

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& option_goal : option_visited_) {
    double x_diff = fabs(goal.x - option_goal.x);
    double y_diff = fabs(goal.y - option_goal.y);

    if (x_diff < tolerance * map_resolution_ &&
        y_diff < tolerance * map_resolution_)
      return true;
  }
  return false;
}

void Explorer::visualizeGoal(decision_maker::NavOption& option){
  // Publish the marker
  std_msgs::msg::ColorRGBA magenta;
  magenta.r = 1.0;
  magenta.g = 0;
  magenta.b = 1.0;
  magenta.a = 1.0;

  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = map_frame_;
  goal_marker.header.stamp = node_->get_clock()->now();
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.id = 0;
  goal_marker.pose.position = option.location;
  double scale = 1.5;
  goal_marker.scale.x = scale;
  goal_marker.scale.y = scale;
  goal_marker.scale.z = scale;
  goal_marker.points = {};
  goal_marker.color = magenta;
  marker_goal_publisher_->publish(goal_marker);
}

}  // namespace explorerr