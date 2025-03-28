/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_manager/path_manager.h"
#include "path_manager/common.h"
#include "task_manager/decco_utilities.h"
#include "messages_88/srv/request_goal.hpp"
#include "messages_88/srv/request_path.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace path_manager
{

template <typename P>
tf2::Vector3 toTfVector3(const P& point) {
  return tf2::Vector3(point.x, point.y, point.z);
}

inline double distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
  return distance(a.pose.position, b.pose.position);
}

PathManager::PathManager()
  : Node("path_manager")
  , tf_listener_(nullptr)
  , goal_init_(false)
  , adjustment_margin_(0.5)
  , setpoint_acceptance_radius_(0.5)
  , goal_acceptance_radius_(2.0)
  , obstacle_dist_threshold_(2.0)
  , percent_above_(-1.0)
  , percent_above_threshold_(0.01)
  , mavros_map_frame_("map")
  , adjust_goal_altitude_(false)
  , adjust_setpoint_(false)
  , adjust_altitude_volume_(false)
  , do_slam_(true)
  , target_altitude_(3.0)
  , planning_horizon_(6.0)
  , velocity_setpoint_speed_(0.5)
  , explorable_goals_(true)
{

  // Params
  this->declare_parameter("setpoint_acceptance_radius", setpoint_acceptance_radius_);
  this->declare_parameter("goal_acceptance_radius", goal_acceptance_radius_);
  this->declare_parameter("obstacle_dist_threshold", obstacle_dist_threshold_);
  this->declare_parameter("mavros_map_frame", mavros_map_frame_);
  this->declare_parameter("adjust_goal", adjust_goal_altitude_);
  this->declare_parameter("adjust_setpoint", adjust_setpoint_);  
  this->declare_parameter("adjust_altitude_volume", adjust_altitude_volume_);  
  this->declare_parameter("raw_goal_topic", "/goal_raw");
  this->declare_parameter("percent_above_thresh", percent_above_threshold_);
  this->declare_parameter("default_alt", target_altitude_);
  this->declare_parameter("do_slam", do_slam_);
  this->declare_parameter("planning_horizon", planning_horizon_);
  this->declare_parameter("velocity_setpoint_speed", velocity_setpoint_speed_);
  this->declare_parameter("explorable_goals", explorable_goals_);

  std::string raw_goal_topic;
  // Params
  this->get_parameter("setpoint_acceptance_radius", setpoint_acceptance_radius_);
  this->get_parameter("goal_acceptance_radius", goal_acceptance_radius_);
  this->get_parameter("obstacle_dist_threshold", obstacle_dist_threshold_);
  this->get_parameter("mavros_map_frame", mavros_map_frame_);
  this->get_parameter("adjust_goal", adjust_goal_altitude_);
  this->get_parameter("adjust_setpoint", adjust_setpoint_);  
  this->get_parameter("adjust_altitude_volume", adjust_altitude_volume_);  
  this->get_parameter("raw_goal_topic", raw_goal_topic);
  this->get_parameter("percent_above_thresh", percent_above_threshold_);
  this->get_parameter("default_alt", target_altitude_);
  this->get_parameter("do_slam", do_slam_);
  this->get_parameter("planning_horizon", planning_horizon_);
  planning_horizon_ -= 1; // Subtract 1 for a safety margin to ensure the segmented goals are fully inside the regional cloud
  this->get_parameter("velocity_setpoint_speed", velocity_setpoint_speed_);
  this->get_parameter("explorable_goals", explorable_goals_);
  
  // Subscribers
  position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&PathManager::positionCallback, this, _1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/search_node/trajectory_position", 10, std::bind(&PathManager::setCurrentPath, this, _1));
  percent_above_sub_ = this->create_subscription<std_msgs::msg::Float32>("/pcl_analysis/percent_above", 10, std::bind(&PathManager::percentAboveCallback, this, _1));
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", 10, std::bind(&PathManager::pointCloudCallback, this, _1));
  raw_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(raw_goal_topic, 10, std::bind(&PathManager::rawGoalCallback, this, _1));
  clicked_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathManager::rawGoalCallback, this, _1));


  // Publishers
  mavros_setpoint_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
  setpoint_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("setpoint_viz", 10);
  actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("actual_path", 10);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);

  cloud_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

PathManager::~PathManager() {}

// Sets the current position and checks if the current setpoint has been reached
void PathManager::positionCallback(const geometry_msgs::msg::PoseStamped &msg) {
  // Update position
  current_pos_ = msg;

  current_pos_.header.frame_id = mavros_map_frame_;
  actual_path_.poses.push_back(current_pos_);
  actual_path_pub_->publish(actual_path_);

  updateGoal();

  updateSetpoint();
}

void PathManager::updateGoal() {
  // Publish next goal when we have reached current one
  if (sub_goals_.size() > 0) {
    if (isCloseToGoal()) {

      // Remove sub goal from list, and set new current goal.
      RCLCPP_INFO(this->get_logger(), "Sub-goal complete");
      sub_goals_.erase(sub_goals_.begin());
      if (sub_goals_.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Final goal complete");
        return;
      }

      current_goal_ = sub_goals_.at(0);

      if (goal_init_ && adjust_goal_altitude_) {
        adjustGoalAltitude(current_goal_);
      }
      // Republish goal here regardless of if it needs adjustment
      publishGoal(current_goal_);
    }
  }
}

void PathManager::updateSetpoint() {

  // No path, nothing to do
  if (path_.size() < 1)
    return;

  // Check if we are close enough to current setpoint to get the next part of the
  // path. Do as a while loop so that we publish the furthest setpoint that is still within the acceptance radius
  while (isCloseToSetpoint() || isCloserThanSetpoint()) {
    // Get furthest ahead setpoint and set as current setpoint
    path_.erase(path_.begin());
    if (path_.size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Path complete");
      return;
    }

    current_setpoint_ = path_[0];

    // Check again, and if outside the range, then publish setpoint outside range. 
    if (!isCloseToSetpoint() && !isCloserThanSetpoint()) {
      publishSetpoint(true);
    }
  }

  // If last published setpoint is older than 1 second, republish it. This is to get the drone "un-stuck" if it goes off path
  if (last_published_setpoint_time_.seconds() > 0 && this->get_clock()->now() - last_published_setpoint_time_ > 1s) {
    RCLCPP_INFO(this->get_logger(), "Path manager republishing last setpoint");
    publishSetpoint(false);
  }
}

bool PathManager::isCloserThanSetpoint() {
  return distance(path_.back(), current_pos_) < distance(path_.back(), current_setpoint_);
}

void PathManager::percentAboveCallback(const std_msgs::msg::Float32 &msg) {
  percent_above_ = msg.data;
}

void PathManager::publishGoal(geometry_msgs::msg::PoseStamped goal) {
  current_goal_.header.frame_id = mavros_map_frame_; // Path planner doesn't return headers, it seems
  // Determine if open area and path planner is needed
  bool open_area = percent_above_ < percent_above_threshold_ && percent_above_ >= 0.0f;

  if (open_area || !do_slam_) {
    // Open area, request goal directly not through path planner

    if (adjust_altitude_volume_) {
      double altitude;
      adjustAltitudeVolume(current_goal_.pose.position, altitude);
      current_goal_.pose.position.z = altitude;
    }

    // Get goal data
    auto direction_vec = subtractPoints(current_goal_.pose.position, current_pos_.pose.position);
    double yaw_target = atan2(direction_vec.y, direction_vec.x);

    // Publish goal
    mavros_msgs::msg::PositionTarget setpoint;
    setpoint.header.frame_id = mavros_map_frame_;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.position = current_goal_.pose.position;
    setpoint.yaw = yaw_target;
    setpoint.coordinate_frame = setpoint.FRAME_LOCAL_NED;
    setpoint.type_mask |= setpoint.IGNORE_AFX | setpoint.IGNORE_AFY | setpoint.IGNORE_AFZ | 
                          setpoint.IGNORE_VX | setpoint.IGNORE_VY | setpoint.IGNORE_VZ | setpoint.IGNORE_YAW_RATE;

    RCLCPP_INFO(this->get_logger(), "Path manager publishing MAVROS goal: [%f, %f, %f]", current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z);
    mavros_setpoint_raw_pub_->publish(setpoint);

    // Publish setpoint vizualizer (does not include velocity, potential TODO)
    geometry_msgs::msg::PoseStamped viz_msg;
    viz_msg.header.stamp = this->get_clock()->now();
    viz_msg.header.frame_id = mavros_map_frame_;
    viz_msg.pose.position = setpoint.position;
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, setpoint.yaw);
    tf2::convert(setpoint_q, viz_msg.pose.orientation);

    setpoint_viz_pub_->publish(viz_msg);
  }
  else {
    if (explorable_goals_)
      current_goal_ = requestExplorableGoal(current_goal_);

    if (adjust_altitude_volume_) {
      double altitude;
      adjustAltitudeVolume(current_goal_.pose.position, altitude);
      current_goal_.pose.position.z = altitude;
    }
    
    // Request path from path planner
    requestPath(current_goal_);
  }
}

geometry_msgs::msg::PoseStamped PathManager::requestExplorableGoal(const geometry_msgs::msg::PoseStamped goal) {
  std::shared_ptr<rclcpp::Node> get_exploregoal_node = rclcpp::Node::make_shared("get_exploregoal_node");
  auto get_goal_client = get_exploregoal_node->create_client<messages_88::srv::RequestGoal>("/explorable_goal");
  auto goal_req = std::make_shared<messages_88::srv::RequestGoal::Request>();
  goal_req->input_goal = goal.pose;

  auto result = get_goal_client->async_send_request(goal_req);
  geometry_msgs::msg::Pose output_goal;
  if (rclcpp::spin_until_future_complete(get_exploregoal_node, result, 1s) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {

      try
      {
          output_goal = result.get()->output_goal;
          RCLCPP_INFO(this->get_logger(), "Got explore goal");
      }
      catch (const std::exception &e)
      {
          output_goal = goal.pose;
          RCLCPP_ERROR(this->get_logger(), "Failed to get explore goal result, using unmodified goal");
      }
      
  } else {
      output_goal = goal.pose;
      RCLCPP_ERROR(this->get_logger(), "Failed to get explore goal, using unmodified goal");
  }
  geometry_msgs::msg::PoseStamped output = goal;
  output.pose = output_goal;
  return output;
}

bool PathManager::requestPath(const geometry_msgs::msg::PoseStamped goal) {

  // Request path from path planner
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("request_path_client");
  rclcpp::Client<messages_88::srv::RequestPath>::SharedPtr client =
    node->create_client<messages_88::srv::RequestPath>("/path_planner/request_path");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for path planner service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Path planner service not available, waiting again...");
  }

  auto request = std::make_shared<messages_88::srv::RequestPath::Request>();
  request->goal = goal;

  RCLCPP_INFO(this->get_logger(), "Path manager requesting PLANNER path for goal: [%f, %f, %f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  
  // Publish setpoint vizualizer
  auto direction_vec = subtractPoints(current_goal_.pose.position, current_pos_.pose.position);
  double yaw_target = atan2(direction_vec.y, direction_vec.x);
  geometry_msgs::msg::PoseStamped viz_msg;
  viz_msg.header.stamp = this->get_clock()->now();
  viz_msg.header.frame_id = mavros_map_frame_;
  viz_msg.pose.position = goal.pose.position;
  tf2::Quaternion setpoint_q;
  setpoint_q.setRPY(0.0, 0.0, yaw_target);
  tf2::convert(setpoint_q, viz_msg.pose.orientation);

  setpoint_viz_pub_->publish(viz_msg);

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result, 10s) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res_ptr = result.get();
    if (res_ptr->success) {
      // setCurrentPath(res_ptr->path);
      return true;
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Path planner failed to create valid path");
    }

  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call path planner service");
  }

  return false;
}

void PathManager::adjustAltitudeVolume(const geometry_msgs::msg::Point &map_position, double &target_altitude) {
  std::shared_ptr<rclcpp::Node> get_elevation_node = rclcpp::Node::make_shared("get_elevation_node");
  auto get_elevation_client = get_elevation_node->create_client<messages_88::srv::GetMapData>("/task_manager/get_map_data");
  auto elevation_req = std::make_shared<messages_88::srv::GetMapData::Request>();
  elevation_req->map_position = map_position;
  elevation_req->adjust_params = true;
  elevation_req->width = 8;
  elevation_req->height = 8;

  auto result = get_elevation_client->async_send_request(elevation_req);
  if (rclcpp::spin_until_future_complete(get_elevation_node, result, 1s) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {

      try
      {
          target_altitude = result.get()->target_altitude;
      }
      catch (const std::exception &e)
      {
          target_altitude = target_altitude_;
          RCLCPP_ERROR(this->get_logger(), "Failed to get elevation result, using default alt of %fm", target_altitude);
      }
      
  } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get elevation");
  }
}

void PathManager::pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg) {
  pcl::fromROSMsg(msg, *cloud_map_);

  if (goal_init_ && adjust_goal_altitude_) {
    if (adjustGoalAltitude(current_goal_)) {
      publishGoal(current_goal_);
    }
  }
}

// Currently unused but may want in future
/* void PathManager::livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {


  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.header.frame_id = msg->header.frame_id;
  cloud.header.stamp = msg->header.stamp.toNSec() / 1000; // PCL type stamp is in microseconds
  
  int plsize = msg->point_num;
  cloud.reserve(plsize);

  for (unsigned i = 0; i < plsize; i++) {
    pcl::PointXYZ added_point;

    added_point.x = msg->points[i].x;
    added_point.y = msg->points[i].y;
    added_point.z = msg->points[i].z;

    cloud.points.push_back(added_point);
  }

  cloud_map_ = transformCloudToMapFrame(cloud);
  if (goal_init_ && adjust_goal_altitude_)
    adjustGoalAltitude(current_goal_);
} */

void PathManager::rawGoalCallback(const geometry_msgs::msg::PoseStamped &msg) {

  RCLCPP_INFO(this->get_logger(), "Received raw goal [%f, %f, %f]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

  sub_goals_ = segmentGoal(msg);
  current_goal_ = sub_goals_.at(0);

  if (adjust_goal_altitude_) {
    adjustGoalAltitude(current_goal_);
  }

  publishGoal(current_goal_);
}

std::vector<geometry_msgs::msg::PoseStamped> PathManager::segmentGoal(geometry_msgs::msg::PoseStamped goal) {

  double distance = decco_utilities::distance_xy(current_pos_.pose.position, goal.pose.position);

  int num_segments = std::ceil(distance / planning_horizon_);

  std::vector<geometry_msgs::msg::PoseStamped> sub_goals;

  // Calculate and store the interpolated points
  for (int i = 1; i <= num_segments; ++i) {
    geometry_msgs::msg::PoseStamped Pi;
    double t = static_cast<double>(i) / static_cast<double>(num_segments);
    Pi.pose.position.x = current_pos_.pose.position.x + t * (goal.pose.position.x - current_pos_.pose.position.x);
    Pi.pose.position.y = current_pos_.pose.position.y + t * (goal.pose.position.y - current_pos_.pose.position.y);
    Pi.pose.position.z = current_pos_.pose.position.z + t * (goal.pose.position.z - current_pos_.pose.position.z);

    sub_goals.push_back(Pi);
  }
  return sub_goals;
}

bool PathManager::adjustGoalAltitude(geometry_msgs::msg::PoseStamped goal) {

  float closest_point_distance;
  pcl::PointXYZ closest_point;

  float original_alt = goal.pose.position.z;
  float max_alt, min_alt;

  bool goal_above_max = false;
  bool goal_below_min = false;

  if (!this->get_parameter("/task_manager/max_alt", max_alt))
    RCLCPP_WARN(this->get_logger(), "Drone state manager cannot get max altitude param");

  if (!this->get_parameter("/task_manager/min_alt", min_alt))
    RCLCPP_WARN(this->get_logger(), "Drone state manager cannot get min altitude param");

  // This is initially a little complicated to understand. What this is doing is
  // alternating checking a point above the original altitude or below it
  // to see if the goal point is within the obstacle dist threshold to nearest pcl point.

  // Increasing iterations will check points further and further away from the
  // original altitude, and we will use the closest point that is OK. 
  unsigned i = 0;
  bool goal_ok = false;
  while (!goal_ok) {

    // Break out of loop if we have checked all options and none are OK
    if (goal_above_max && goal_below_min) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Path Manager: Acceptable goal not found");
      break;
    }

    // Check original altitude, do nothing here
    if (i == 0) {
    }
    // Check point above original alt
    else if (i % 2 == 1) {
      float alt_adjusted_above = original_alt + adjustment_margin_ * (i + 1) / 2;

      if (alt_adjusted_above <= max_alt)
        goal.pose.position.z = alt_adjusted_above;
      else {
        goal_above_max = true;
        i++;
        continue;
      }
    }
    // Check point below original alt
    else {
      float alt_adjusted_below = original_alt - adjustment_margin_ * i / 2;

      if (alt_adjusted_below >= min_alt)
        goal.pose.position.z = alt_adjusted_below;
      else {
        goal_below_min = true;
        i++;
        continue;
      }
    }

    // Check to see if goal point is in obstacle dist threshold, if not, publish it.
    findClosestPointInCloud(cloud_map_, goal.pose.position, closest_point, closest_point_distance);
    if (closest_point_distance > obstacle_dist_threshold_) {
      goal_ok = true;
      goal_init_ = true;
      if (current_goal_ != goal) {
        current_goal_ = goal;
        // Returning true indicates that goal has been adjusted
        return true;
      }
    }

    i++;
  }
  return false;
}

void PathManager::setCurrentPath(const nav_msgs::msg::Path &path) {

  RCLCPP_INFO(this->get_logger(), "Received path, target position [%f, %f, %f]",
                                  path.poses.back().pose.position.x, path.poses.back().pose.position.y, path.poses.back().pose.position.z);

  std::vector<geometry_msgs::msg::PoseStamped> poses = path.poses;

  path_.clear();

  if (poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  for (unsigned i = 0; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }

  current_setpoint_ = path_[0];

  // Publish first setpoint
  publishSetpoint(true);
}

void PathManager::publishSetpoint(bool use_velocity) {

  if (path_.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No path to publish setpoint");
    return;
  }

  if (adjust_setpoint_)
    adjustSetpoint();

  mavros_msgs::msg::PositionTarget msg;

  msg.header.frame_id = mavros_map_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.position = current_setpoint_.pose.position;

  int viable_pts_ahead = std::min(5, static_cast<int>(path_.size()));
  // Create vector pointing from current setpoint to point ahead on path, using 5 points ahead as rough estimation
  // TODO play with this number
  auto vec = subtractPoints(path_[viable_pts_ahead].pose.position, current_setpoint_.pose.position);

  // Only set yaw target if vector is non-zero
  if (fabs(vec.x) > FLT_EPSILON || fabs(vec.y) > FLT_EPSILON) {
    yaw_target_ = atan2(vec.y, vec.x);
  }

  msg.yaw = yaw_target_;

  if (use_velocity) {

    // Normalize vector and set yaw after checking for 0 on vector length
    geometry_msgs::msg::Vector3 unit_vec;
    double mag = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if (mag > FLT_EPSILON) {
      unit_vec.x = vec.x / mag;
      unit_vec.y = vec.y / mag;
      unit_vec.z = vec.z / mag;
    }
    else {
      unit_vec.x = 0.0;
      unit_vec.y = 0.0;
      unit_vec.z = 0.0;
    }
  
    // Determine absolute drone speed, initially set to velocity setpoint speed param
    double speed = velocity_setpoint_speed_;

    // Calculate distance to end of path. As we approach path end (within threshold), reduce speed
    // using v^2 = u^2 + 2*a*s, with safe estimated acceleration of 2 m/s/s
    double dist_to_end = distance(current_setpoint_, path_.back());
    double acceleration = 2.0;
    double threshold = velocity_setpoint_speed_ * velocity_setpoint_speed_ / (2.0 * acceleration);
    if (dist_to_end < threshold) {
      speed *= dist_to_end / threshold;
    }

    // If at the end of the path, set speed to 0.
    if (path_.size() == 0) {
      speed = 0.0;
    }

    msg.velocity.x = unit_vec.x * speed;
    msg.velocity.y = unit_vec.y * speed;
    msg.velocity.z = unit_vec.z * speed;
  }

  // Now fill in other elements of message
  msg.coordinate_frame = msg.FRAME_LOCAL_NED;
  msg.type_mask |= msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ | msg.IGNORE_YAW_RATE;

  mavros_setpoint_raw_pub_->publish(msg);

  // Update last published setpoint time
  last_published_setpoint_time_ = this->get_clock()->now();

  // Publish setpoint vizualizer (does not include velocity, potential TODO)
  geometry_msgs::msg::PoseStamped viz_msg;
  viz_msg.header.stamp = this->get_clock()->now();
  viz_msg.header.frame_id = mavros_map_frame_;
  viz_msg.pose.position = msg.position;
  tf2::Quaternion setpoint_q;
  setpoint_q.setRPY(0.0, 0.0, msg.yaw);
  tf2::convert(setpoint_q, viz_msg.pose.orientation);

  setpoint_viz_pub_->publish(viz_msg);
}

bool PathManager::isCloseToGoal() {
  return distance(current_pos_, current_goal_) < goal_acceptance_radius_;
}

bool PathManager::isCloseToSetpoint() {
  return distance(current_pos_, current_setpoint_) < setpoint_acceptance_radius_;
}

void PathManager::adjustSetpoint() {

  // Check PCL points for proximity to setpoint, and find closest point in PCL to setpoint.
  float closest_point_distance;
  pcl::PointXYZ closest_point;

  findClosestPointInCloud(cloud_map_, current_setpoint_.pose.position, closest_point, closest_point_distance);

  // Check if setpoint is within our obstacle distance threshold to its closest point in the PCL.
  // If so, adjust setpoint so that it is outside threshold, but as near as possible to original setpoint
  if (closest_point_distance < obstacle_dist_threshold_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint inside obstacle distance threshold, adjusting setpoint");
    float scale_factor = obstacle_dist_threshold_ / closest_point_distance;

    float dist_x = current_setpoint_.pose.position.x - closest_point.x;
    float dist_y = current_setpoint_.pose.position.y - closest_point.y;
    float dist_z = current_setpoint_.pose.position.z - closest_point.z;

    current_setpoint_.pose.position.x = closest_point.x + dist_x * scale_factor;
    current_setpoint_.pose.position.y = closest_point.y + dist_y * scale_factor;
    current_setpoint_.pose.position.z = closest_point.z + dist_z * scale_factor;
  }
}

void PathManager::findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::Point point_in, 
                                           pcl::PointXYZ &closest_point, float &closest_point_distance) {

  closest_point_distance = INFINITY;
  for (auto &point : *cloud) {
    float dist_x = point_in.x - point.x;
    float dist_y = point_in.y - point.y;
    float dist_z = point_in.z - point.z;

    float total_dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

    if (total_dist < closest_point_distance) {
      closest_point_distance = total_dist;
      closest_point = point;
    }
  }
}

} // namespace path_manager
