/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_manager/path_manager.h"
#include "path_manager/common.h"
#include "task_manager/decco_utilities.h"
#include "messages_88/srv/request_path.hpp"

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
  , velocity_setpoint_(false)
  , velocity_setpoint_speed_(1.0)
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
  this->declare_parameter("velocity_setpoint", velocity_setpoint_);
  this->declare_parameter("velocity_setpoint_speed", velocity_setpoint_speed_);

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
  this->get_parameter("velocity_setpoint", velocity_setpoint_);
  this->get_parameter("velocity_setpoint_speed", velocity_setpoint_speed_);
  
  // Subscribers
  position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&PathManager::positionCallback, this, _1));
  percent_above_sub_ = this->create_subscription<std_msgs::msg::Float32>("/pcl_analysis/percent_above", 1, std::bind(&PathManager::percentAboveCallback, this, _1));
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", 1, std::bind(&PathManager::pointCloudCallback, this, _1));
  raw_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(raw_goal_topic, 1, std::bind(&PathManager::rawGoalCallback, this, _1));

  // Publishers
  mavros_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
  mavros_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("actual_path", 10);
}

PathManager::~PathManager() {}

// Sets the current position and checks if the current setpoint has been reached
void PathManager::positionCallback(const geometry_msgs::msg::PoseStamped &msg) {
  // Update position
  last_pos_ = msg;

  last_pos_.header.frame_id = mavros_map_frame_;
  actual_path_.poses.push_back(last_pos_);
  actual_path_pub_->publish(actual_path_);

  // Publish next goal when we have reached current one
  if (sub_goals_.size() > 1 && isCloseToGoal()) {

    // Remove sub goal from list, and set new current goal. 
    sub_goals_.erase(sub_goals_.begin());
    current_goal_ = sub_goals_.at(0);

    // Update yaw target for current goal
    auto direction_vec = subtractPoints(current_goal_.pose.position, last_pos_.pose.position);
    yaw_target_ = atan2(direction_vec.y, direction_vec.x);

    if (adjust_altitude_volume_) {
      double altitude;
      adjustAltitudeVolume(current_goal_.pose.position, altitude);
      current_goal_.pose.position.z = altitude;
    }
    if (goal_init_ && adjust_goal_altitude_) {
      adjustGoalAltitude(current_goal_);
    }
    // Republish goal here regardless of if it needs adjustment
    publishGoal(current_goal_);
  }


  // Check if we are close enough to current setpoint to get the next part of the
  // path. Do as a while loop so that we publish the furthest setpoint that is still within the acceptance radius
  if (path_.size() > 0) {
    publishSetpoint();
    
    if (isCloseToSetpoint()) {
      last_setpoint_ = current_setpoint_;    
      current_setpoint_ = path_[0];
      path_.erase(path_.begin());
    }
    
  }
}

void PathManager::percentAboveCallback(const std_msgs::msg::Float32 &msg) {
  percent_above_ = msg.data;
}

void PathManager::publishGoal(geometry_msgs::msg::PoseStamped goal) {
  goal.header.frame_id = mavros_map_frame_; // Path planner doesn't return headers, it seems
  // Determine if open area and path planner is needed
  RCLCPP_INFO(this->get_logger(), "Path manager publishing goal: [%f, %f, %f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  bool open_area = percent_above_ < percent_above_threshold_ && percent_above_ >= 0.0f;

  if (open_area || !do_slam_) {
    // Publish mavros setpoint directly including yaw target if in an open area or not using slam
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, yaw_target_);
    tf2::convert(setpoint_q, goal.pose.orientation);
    mavros_setpoint_pub_->publish(goal);
  }
  else {
    // Request path with 3 attempts
    for (unsigned i = 0; i < 3; i++) {
      if (requestPath(goal))
        break;
    }
  }
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

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res_ptr = result.get();

    // If successful, set path
    if (res_ptr->success) {
      setCurrentPath(res_ptr->path);
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
  if (rclcpp::spin_until_future_complete(get_elevation_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {

      try
      {
          target_altitude = result.get()->target_altitude;
          RCLCPP_INFO(this->get_logger(), "Got elevation %f", target_altitude);
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
  pcl::fromROSMsg(msg, cloud_map_);

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

  RCLCPP_INFO(this->get_logger(), "Received goal");

  sub_goals_ = segmentGoal(msg);
  
  current_goal_ = sub_goals_.at(0);
  auto direction_vec = subtractPoints(current_goal_.pose.position, last_pos_.pose.position);
  yaw_target_ = atan2(direction_vec.y, direction_vec.x);

  if (adjust_goal_altitude_) {
    adjustGoalAltitude(current_goal_);
  }
  if (adjust_altitude_volume_) {
    double altitude;
    adjustAltitudeVolume(current_goal_.pose.position, altitude);
    current_goal_.pose.position.z = altitude;
  }

  publishGoal(current_goal_);
}

std::vector<geometry_msgs::msg::PoseStamped> PathManager::segmentGoal(geometry_msgs::msg::PoseStamped goal) {

  double distance = decco_utilities::distance_xy(last_pos_.pose.position, goal.pose.position);

  int num_segments = std::ceil(distance / planning_horizon_);

  std::vector<geometry_msgs::msg::PoseStamped> sub_goals;

  // Calculate and store the interpolated points
  for (int i = 1; i <= num_segments; ++i) {
    geometry_msgs::msg::PoseStamped Pi;
    double t = static_cast<double>(i) / static_cast<double>(num_segments);
    Pi.pose.position.x = last_pos_.pose.position.x + t * (goal.pose.position.x - last_pos_.pose.position.x);
    Pi.pose.position.y = last_pos_.pose.position.y + t * (goal.pose.position.y - last_pos_.pose.position.y);
    Pi.pose.position.z = last_pos_.pose.position.z + t * (goal.pose.position.z - last_pos_.pose.position.z);

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

pcl::PointCloud<pcl::PointXYZ> PathManager::transformCloudToMapFrame(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
  pcl::PointCloud<pcl::PointXYZ> cloud_out;

  geometry_msgs::msg::TransformStamped pcl_map_tf;
  try {
    pcl_map_tf = tf_buffer_->lookupTransform(mavros_map_frame_, cloud_in.header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Path Manager: %s",ex.what());
    return cloud_out;
  }
  pcl_ros::transformPointCloud(cloud_in, cloud_out, pcl_map_tf);

  return cloud_out;
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

  last_setpoint_ = path_[0];
  current_setpoint_ = path_[1];

  // Calculate orientation to point vehicle towards final destination
  geometry_msgs::msg::PoseStamped final_setpoint = path_.back();
  auto direction_vec = subtractPoints(final_setpoint.pose.position, last_pos_.pose.position);
  yaw_target_ = atan2(direction_vec.y, direction_vec.x);

  // Publish first setpoint
  publishSetpoint();
}

void PathManager::publishSetpoint() {

  if (adjust_setpoint_)
    adjustSetpoint();

  auto setpoint = current_setpoint_;  // The intermediate position sent to Mavros

  // Fill setpoint data
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = mavros_map_frame_;
  setpoint.pose = current_setpoint_.pose;

  tf2::Quaternion setpoint_q;
  setpoint_q.setRPY(0.0, 0.0, yaw_target_);
  tf2::convert(setpoint_q, setpoint.pose.orientation);

  // Publish setpoint to Mavros
  if (!velocity_setpoint_) {
    // Use position setpoint
    mavros_setpoint_pub_->publish(setpoint);
  }
  else {
    // Use velocity setpoint. TODO improve trajectory planning - I have experimented with some methods but they have been unreliable so far.
    // Potential to reincorporate in the future, using acceleration, curvature lookahead, etc. For now just using the simplest method to have something. 

    // Set velocity vector from difference in current location to point on path.
    geometry_msgs::msg::Vector3 vec;
    vec.x = current_setpoint_.pose.position.x - last_pos_.pose.position.x;
    vec.y = current_setpoint_.pose.position.y - last_pos_.pose.position.y;
    vec.z = current_setpoint_.pose.position.z - last_pos_.pose.position.z;

    // Normalize vector
    double mag = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if (mag == 0.0) {
      RCLCPP_INFO(this->get_logger(), "Invalid pose/setpoint, not setting velocity command");
      return;
    }

    geometry_msgs::msg::Vector3 unit_vec;
    unit_vec.x = vec.x / mag;
    unit_vec.y = vec.y / mag;
    unit_vec.z = vec.z / mag;


    // Determine absolute drone speed, initially set to velocity setpoint speed param
    double speed = velocity_setpoint_speed_;

    // Calculate distance to end of path. As we approach path end (within threshold), reduce speed
    double dist_to_end = distance(last_pos_, path_.back());
    double threshold = velocity_setpoint_speed_ * 2.0;
    if (dist_to_end < threshold) {
      speed *= dist_to_end / threshold;
    }

    // If at the end of the path, set speed to 0.
    if (path_.size() == 1) {
      speed = 0.0;
    }

    // Apply speed to direction unit vec, and publish as velocity setpoing
    geometry_msgs::msg::TwistStamped setpoint_vel;
    setpoint_vel.twist.linear.x = unit_vec.x * speed;
    setpoint_vel.twist.linear.y = unit_vec.y * speed;
    setpoint_vel.twist.linear.z = unit_vec.z * speed;

    mavros_velocity_pub_->publish(setpoint_vel);
  }
}

bool PathManager::isCloseToGoal() {
  return distance(last_pos_, current_goal_) < goal_acceptance_radius_;
}

bool PathManager::isCloseToSetpoint() {
  return distance(last_pos_, current_setpoint_) < setpoint_acceptance_radius_;
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

void PathManager::findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::msg::Point point_in, 
                                           pcl::PointXYZ &closest_point, float &closest_point_distance) {

  closest_point_distance = INFINITY;
  for (auto &point : cloud) {
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
