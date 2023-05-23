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
  , default_speed_(2.0)
  , position_received_(false)
  , goal_received_(false)
  , cmdloop_dt_(0.1)
  , path_topic_("/search_node/trajectory_position")
  , frame_id_("map")
{
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("default_speed", default_speed_, default_speed_);
    
  position_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &PathToMavros::positionCallback, this);
  path_sub_ = nh_.subscribe(path_topic_, 1, &PathToMavros::setCurrentPath, this);

  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("actual_path", 10);

  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&PathToMavros::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

// Sets the current position and checks if the current goal has been reached
void PathToMavros::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;
  setPose(last_pos_);

  // Check if a new goal is needed
//   if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    last_pos_.header.frame_id = frame_id_;
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_.publish(actual_path_);
//   }

  position_received_ = true;
  if (path_.size() == 0) {
    goal_received_ = false;
  }

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    // Transform yaw_diff to [0, 2*pi]
    yaw_diff -= std::floor(yaw_diff / (2 * M_PI)) * (2 * M_PI);
    double max_yaw_diff = M_PI / 1.0;
    if (yaw_diff < max_yaw_diff || yaw_diff > 2 * M_PI - max_yaw_diff) {
      // If we are facing the right direction, then pop the first point of the
      // path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      path_.erase(path_.begin());
    }
  }
}

void PathToMavros::setCurrentPath(const nav_msgs::Path::ConstPtr &path) {
  std::vector<geometry_msgs::PoseStamped> poses = path->poses;
  path_.clear();

  if (poses.size() < 2) {
    ROS_INFO("  Received empty path\n");
    return;
  }
  goal_received_ = true;
  last_goal_ = poses[0];
  current_goal_ = poses[1];

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

// Updates the current pose /and keeps track of the path back
void PathToMavros::setPose(const geometry_msgs::PoseStamped& new_pose) {
  curr_pos_ = new_pose;
  curr_yaw_ = tf::getYaw(new_pose.pose.orientation);
//   Cell curr_cell = Cell(curr_pos_);
//   if (!going_back_ && (path_back_.empty() || curr_cell != path_back_.back())) {
//     // Keep track of where we have been, add current position to path_back_ if
//     // it is different from last one
//     path_back_.push_back(curr_cell);
//   }
}

void PathToMavros::cmdLoopCallback(const ros::TimerEvent& event) {
    if (!goal_received_) return;
//   hover_ = false;

  // Check if all information was received
//   ros::Time now = ros::Time::now();

//   ros::Duration since_last_cloud = now - last_wp_time_;
//   ros::Duration since_start = now - start_time_;

//   avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void PathToMavros::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf::Vector3 vec = toTfVector3(subtractPoints(current_goal_.pose.position, last_pos_.pose.position));
  // TODO add a version of this commented section back later, adjust speed based on risk
//   if (global_planner_.use_speedup_heuristics_) {
//     Cell cur_cell =
//         global_planner::Cell(last_pos_.pose.position.x, last_pos_.pose.position.y, last_pos_.pose.position.z);
//     double cur_risk = std::sqrt(global_planner_.getRisk(cur_cell));
//     if (cur_risk >= global_planner_.risk_threshold_risk_based_speedup_) {  // If current risk is too high(more than
//                                                                            // risk_threshold_risk_based_speedup_), set
//                                                                            // speed as low to stable flight.
//       speed_ = global_planner_.default_speed_;
//     } else {  // If current risk is low, speed up for fast flight.
//       speed_ = global_planner_.default_speed_ +
//                (global_planner_.max_speed_ - global_planner_.default_speed_) * (1 - cur_risk);
//     }
//   } else {  // If risk based speed up is not activated, use default_speed_.
    // speed_ = default_speed_;
//   }

  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < 1.0 ? vec.length() : default_speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();
  geometry_msgs::Quaternion quat = setpoint.pose.orientation;
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);
  tf_quat.normalize();
  setpoint.pose.orientation.x = tf_quat.getX();
  setpoint.pose.orientation.y = tf_quat.getY();
  setpoint.pose.orientation.z = tf_quat.getZ();
  setpoint.pose.orientation.w = tf_quat.getW();

//   // Publish setpoint for vizualization
//   current_waypoint_publisher_.publish(setpoint);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
}

bool PathToMavros::isCloseToGoal() { return distance(current_goal_, last_pos_) < default_speed_; }

}