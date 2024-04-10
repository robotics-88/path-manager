/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_to_mavros/path_to_mavros.h"
#include "path_to_mavros/common.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

namespace path_to_mavros
{

template <typename P>
tf::Vector3 toTfVector3(const P& point) {
  return tf::Vector3(point.x, point.y, point.z);
}

inline double distance(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
  return distance(a.pose.position, b.pose.position);
}

PathToMavros::PathToMavros(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , acceptance_radius_(2.0)
  , obstacle_dist_threshold_(2.0)
  , goal_received_(false)
  , path_topic_("/search_node/trajectory_position")
{
  ros::NodeHandle private_nh("~");

  // Params
  private_nh.param<double>("acceptance_radius", acceptance_radius_, acceptance_radius_);
  private_nh.param<double>("obstacle_dist_threshold", obstacle_dist_threshold_, obstacle_dist_threshold_);
  private_nh.param<std::string>("slam_map_frame", slam_map_frame_, "slam_map");
  private_nh.param<std::string>("mavros_map_frame", mavros_map_frame_, "map");

  std::string cloud_topic;
  private_nh.param<std::string>("cloud_topic", cloud_topic, "/livox/lidar");
  
  // Subscribers
  position_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &PathToMavros::positionCallback, this);
  path_sub_ = nh_.subscribe(path_topic_, 1, &PathToMavros::setCurrentPath, this);
  pointcloud_sub_ = nh_.subscribe(cloud_topic, 1, &PathToMavros::pointCloudCallback, this);

  // Publishers
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("actual_path", 10);
}

// Sets the current position and checks if the current goal has been reached
void PathToMavros::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;

  last_pos_.header.frame_id = mavros_map_frame_;
  actual_path_.poses.push_back(last_pos_);
  actual_path_pub_.publish(actual_path_);

  if (path_.size() == 0) {
    goal_received_ = false;
  }

  // Check if we are close enough to current goal to get the next part of the
  // path. Do as a while loop so that we publish the furthest setpoint that is still within the acceptance radius
  while (path_.size() > 0 && isCloseToGoal()) {
    last_goal_ = current_goal_;
    current_goal_ = path_[0];
    path_.erase(path_.begin());
    publishSetpoint();
  }
}

void PathToMavros::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  last_pointcloud_ = *msg;
}

void PathToMavros::setCurrentPath(const nav_msgs::Path::ConstPtr &path) {
  std::vector<geometry_msgs::PoseStamped> poses = path->poses;

  path_.clear();

  if (poses.size() < 2) {
    ROS_WARN("Received empty path\n");
    return;
  }
  goal_received_ = true;

  // Convert path from slam map frame to mavros map frame
  geometry_msgs::TransformStamped transform_slam2mavros;
  std::string transform_error;
  try {
    transform_slam2mavros = tf_buffer_.lookupTransform(mavros_map_frame_, slam_map_frame_, path->header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }
  for (int i = 0; i < poses.size(); ++i) {
    geometry_msgs::PoseStamped pose_transformed;
    tf2::doTransform(poses[i], pose_transformed, transform_slam2mavros);
    
    path_.push_back(pose_transformed);
  }

  last_goal_ = path_[0];
  current_goal_ = path_[1];

  // Calculate orientation to point vehicle towards final destination
  geometry_msgs::PoseStamped final_goal = path_.back();
  auto direction_vec = subtractPoints(final_goal.pose.position, last_pos_.pose.position);
  yaw_target_ = atan2(direction_vec.y, direction_vec.x);

  // Publish first setpoint
  publishSetpoint();
}

void PathToMavros::publishSetpoint() {

  ensureSetpointSafety();

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros

  // Fill setpoint data
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = mavros_map_frame_;
  setpoint.pose = current_goal_.pose;

  tf2::Quaternion setpoint_q;
  setpoint_q.setRPY(0.0, 0.0, yaw_target_);
  tf2::convert(setpoint_q, setpoint.pose.orientation);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
}

bool PathToMavros::isCloseToGoal() {
  return distance(last_pos_, current_goal_) < acceptance_radius_;
}

void PathToMavros::ensureSetpointSafety() {

  // Convert last PCL to mavros frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(last_pointcloud_, *cloud);

  geometry_msgs::TransformStamped pcl_map_tf;
    try {
        pcl_map_tf = tf_buffer_.lookupTransform(mavros_map_frame_, last_pointcloud_.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    pcl_ros::transformPointCloud(*cloud, *cloud_map, pcl_map_tf.transform);

  // Check PCL points for proximity to goal point, and find closest point in PCL to goal point.
  float closest_point_distance = INFINITY;
  pcl::PointXYZ closest_point;
  for (auto &point : *cloud_map) {
    float dist_x = current_goal_.pose.position.x - point.x;
    float dist_y = current_goal_.pose.position.y - point.y;
    float dist_z = current_goal_.pose.position.z - point.z;

    float total_dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

    if (total_dist < closest_point_distance) {
      closest_point_distance = total_dist;
      closest_point = point;
    }
  }

  // Check if goal point is within our obstacle distance threshold to its closest point in the PCL.
  // If so, adjust goal so that it is outside threshold, but as near as possible to original goal
  if (closest_point_distance < obstacle_dist_threshold_) {
    float scale_factor = obstacle_dist_threshold_ / closest_point_distance;

    float dist_x = current_goal_.pose.position.x - closest_point.x;
    float dist_y = current_goal_.pose.position.y - closest_point.y;
    float dist_z = current_goal_.pose.position.z - closest_point.z;

    current_goal_.pose.position.x = closest_point.x + dist_x * scale_factor;
    current_goal_.pose.position.y = closest_point.y + dist_y * scale_factor;
    current_goal_.pose.position.z = closest_point.z + dist_z * scale_factor;
  }
}

} // namespace path_to_mavros
