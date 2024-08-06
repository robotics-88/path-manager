/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_manager/path_manager.h"
#include "path_manager/common.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "pcl_ros/transforms.h"

namespace path_manager
{

template <typename P>
tf::Vector3 toTfVector3(const P& point) {
  return tf::Vector3(point.x, point.y, point.z);
}

inline double distance(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
  return distance(a.pose.position, b.pose.position);
}

PathManager::PathManager(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , path_received_(false)
  , goal_init_(false)
  , adjustment_margin_(0.5)
{
  ros::NodeHandle private_nh("~");

  // Params
  private_nh.param<double>("acceptance_radius", acceptance_radius_, 2.0);
  private_nh.param<double>("obstacle_dist_threshold", obstacle_dist_threshold_, 2.0);
  private_nh.param<std::string>("mavros_map_frame", mavros_map_frame_, "map");
  private_nh.param<bool>("adjust_goal", adjust_goal_, false);

  int lidar_type; // 4 is Mid360, 2 is Velodyne (or sim)
  private_nh.param<int>("lidar_type", lidar_type, 4);

  std::string cloud_topic, raw_goal_topic, path_topic;
  private_nh.param<std::string>("cloud_topic", cloud_topic, "/livox/lidar");
  private_nh.param<std::string>("raw_goal_topic", raw_goal_topic, "/goal_raw");
  private_nh.param<std::string>("path_topic", path_topic, "/search_node/trajectory_position");
  
  // Subscribers
  position_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &PathManager::positionCallback, this);
  path_sub_ = nh_.subscribe(path_topic, 1, &PathManager::setCurrentPath, this);

  pointcloud_sub_ = lidar_type == 4 ? \
        nh_.subscribe(cloud_topic, 1, &PathManager::livoxPointCloudCallback, this) : \
        nh_.subscribe(cloud_topic, 1, &PathManager::pointCloudCallback, this);

  raw_goal_sub_ = nh_.subscribe(raw_goal_topic, 1, &PathManager::rawGoalCallback, this);

  // Publishers
  mavros_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("actual_path", 10);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 10);
}

// Sets the current position and checks if the current setpoint has been reached
void PathManager::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;

  last_pos_.header.frame_id = mavros_map_frame_;
  actual_path_.poses.push_back(last_pos_);
  actual_path_pub_.publish(actual_path_);

  if (path_.size() == 0) {
    path_received_ = false;
  }

  // Check if we are close enough to current setpoint to get the next part of the
  // path. Do as a while loop so that we publish the furthest setpoint that is still within the acceptance radius
  while (path_.size() > 0 && isCloseToSetpoint()) {
    last_setpoint_ = current_setpoint_;
    current_setpoint_ = path_[0];
    path_.erase(path_.begin());
    publishSetpoint();
  }
}

void PathManager::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  cloud_map_ = transformCloudToMapFrame(cloud);
  if (goal_init_ && adjust_goal_)
    adjustGoal(current_goal_);
}

void PathManager::livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {

  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.header.frame_id = msg->header.frame_id;
  cloud.header.stamp = msg->header.stamp.toNSec() / 1000; // PCL type stamp is in microseconds
  
  int plsize = msg->point_num;
  cloud.reserve(plsize);

  for (int i = 0; i < plsize; i++) {
    pcl::PointXYZ added_point;

    added_point.x = msg->points[i].x;
    added_point.y = msg->points[i].y;
    added_point.z = msg->points[i].z;

    cloud.points.push_back(added_point);
  }

  cloud_map_ = transformCloudToMapFrame(cloud);
  if (goal_init_ && adjust_goal_)
    adjustGoal(current_goal_);
}

void PathManager::rawGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (adjust_goal_) {
    adjustGoal(*msg);
  }
  else {
    goal_pub_.publish(*msg);
  }
}

void PathManager::adjustGoal(geometry_msgs::PoseStamped goal) {

  float closest_point_distance;
  pcl::PointXYZ closest_point;

  float original_alt = goal.pose.position.z;
  float max_alt, min_alt;

  bool goal_above_max = false;
  bool goal_below_min = false;

  if (!ros::param::get("/task_manager/max_alt", max_alt))
    ROS_WARN("Drone state manager cannot get max altitude param");

  if (!ros::param::get("/task_manager/min_alt", min_alt))
    ROS_WARN("Drone state manager cannot get min altitude param");

  // This is initially a little complicated to understand. What this is doing is
  // alternating checking a point above the original altitude or below it
  // to see if the goal point is within the obstacle dist threshold to nearest pcl point.

  // Increasing iterations will check points further and further away from the
  // original altitude, and we will use the closest point that is OK. 
  int i = 0;
  bool goal_ok = false;
  while (!goal_ok) {

    // Break out of loop if we have checked all options and none are OK
    if (goal_above_max && goal_below_min) {
      ROS_WARN_THROTTLE(1, "Path Manager: Acceptable goal not found");
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
        goal_pub_.publish(goal);
      }
    }

    i++;
  }
}

pcl::PointCloud<pcl::PointXYZ> PathManager::transformCloudToMapFrame(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
  pcl::PointCloud<pcl::PointXYZ> cloud_out;

  geometry_msgs::TransformStamped pcl_map_tf;
  try {
    pcl_map_tf = tf_buffer_.lookupTransform(mavros_map_frame_, cloud_in.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(10, "Path Manager: %s",ex.what());
    return cloud_out;
  }
  pcl_ros::transformPointCloud(cloud_in, cloud_out, pcl_map_tf.transform);

  return cloud_out;
}

void PathManager::setCurrentPath(const nav_msgs::Path::ConstPtr &path) {
  std::vector<geometry_msgs::PoseStamped> poses = path->poses;

  path_.clear();

  if (poses.size() < 2) {
    ROS_WARN("Received empty path\n");
    return;
  }
  path_received_ = true;

  for (int i = 0; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }

  last_setpoint_ = path_[0];
  current_setpoint_ = path_[1];

  // Calculate orientation to point vehicle towards final destination
  geometry_msgs::PoseStamped final_setpoint = path_.back();
  auto direction_vec = subtractPoints(final_setpoint.pose.position, last_pos_.pose.position);
  yaw_target_ = atan2(direction_vec.y, direction_vec.x);

  // Publish first setpoint
  publishSetpoint();
}

void PathManager::publishSetpoint() {

  ensureSetpointSafety();

  auto setpoint = current_setpoint_;  // The intermediate position sent to Mavros

  // Fill setpoint data
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = mavros_map_frame_;
  setpoint.pose = current_setpoint_.pose;

  tf2::Quaternion setpoint_q;
  setpoint_q.setRPY(0.0, 0.0, yaw_target_);
  tf2::convert(setpoint_q, setpoint.pose.orientation);

  // Publish setpoint to Mavros
  mavros_setpoint_pub_.publish(setpoint);
}

bool PathManager::isCloseToSetpoint() {
  return distance(last_pos_, current_setpoint_) < acceptance_radius_;
}

void PathManager::ensureSetpointSafety() {

  // Check PCL points for proximity to setpoint, and find closest point in PCL to setpoint.
  float closest_point_distance;
  pcl::PointXYZ closest_point;

  findClosestPointInCloud(cloud_map_, current_setpoint_.pose.position, closest_point, closest_point_distance);

  // Check if setpoint is within our obstacle distance threshold to its closest point in the PCL.
  // If so, adjust setpoint so that it is outside threshold, but as near as possible to original setpoint
  if (closest_point_distance < obstacle_dist_threshold_) {
    ROS_WARN_THROTTLE(1, "Setpoint inside obstacle distance threshold, adjusting setpoint");
    float scale_factor = obstacle_dist_threshold_ / closest_point_distance;

    float dist_x = current_setpoint_.pose.position.x - closest_point.x;
    float dist_y = current_setpoint_.pose.position.y - closest_point.y;
    float dist_z = current_setpoint_.pose.position.z - closest_point.z;

    current_setpoint_.pose.position.x = closest_point.x + dist_x * scale_factor;
    current_setpoint_.pose.position.y = closest_point.y + dist_y * scale_factor;
    current_setpoint_.pose.position.z = closest_point.z + dist_z * scale_factor;
  }
}

void PathManager::findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Point point_in, 
                                           pcl::PointXYZ &closest_point, float &closest_point_distance) {

  closest_point_distance = INFINITY;
  for (auto &point : cloud_map_) {
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
