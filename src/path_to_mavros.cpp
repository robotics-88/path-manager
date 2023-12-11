/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_to_mavros/path_to_mavros.h"
#include "path_to_mavros/common.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

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
  , goal_received_(false)
  , path_topic_("/search_node/trajectory_position")
{
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("acceptance_radius", acceptance_radius_, acceptance_radius_);

  private_nh.param<std::string>("slam_map_frame", slam_map_frame_, "slam_map");
  private_nh.param<std::string>("mavros_map_frame", mavros_map_frame_, "map");
    
  position_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &PathToMavros::positionCallback, this);
  path_sub_ = nh_.subscribe(path_topic_, 1, &PathToMavros::setCurrentPath, this);

  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("actual_path", 10);
}

// Sets the current position and checks if the current goal has been reached
void PathToMavros::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;

  // Check if a new goal is needed
//   if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    last_pos_.header.frame_id = mavros_map_frame_;
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_.publish(actual_path_);
//   }

  if (path_.size() == 0) {
    goal_received_ = false;
  }

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {

    last_goal_ = current_goal_;
    current_goal_ = path_[0];
    path_.erase(path_.begin());

    publishSetpoint();
  }
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
  try{
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

  // Publish first setpoint
  publishSetpoint();
}

void PathToMavros::publishSetpoint() {

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros

  // Calculate orientation to point vehilce towards goal
  auto direction_vec = subtractPoints(current_goal_.pose.position, last_pos_.pose.position);
  double yaw = atan2(direction_vec.y, direction_vec.x);
  tf2::Quaternion setpoint_q;
  setpoint_q.setEuler(yaw, 0.0, 0.0);

  // Fill setpoint pose and orientation
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = mavros_map_frame_;
  setpoint.pose = current_goal_.pose;
  tf2::convert(setpoint_q, setpoint.pose.orientation);

//   // Publish setpoint for vizualization
//   current_waypoint_publisher_.publish(setpoint);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
}

// We are considered 'close to goal' when we are closer to the next point than the last point, 
// but with a certain maximum distance of 'acceptance_radius_'
bool PathToMavros::isCloseToGoal() { 
  double dist_to_next_point = distance(last_pos_, current_goal_);
  
  return dist_to_next_point < acceptance_radius_;
}

} // namespace path_to_mavros
