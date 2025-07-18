/*
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "path_manager/path_manager.h"
#include "task_manager/decco_utilities.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace path_manager {

template <typename T> T norm(T x, T y, T z) {
    return sqrt(x * x + y * y + z * z);
}

double dotProduct(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename P> double distance(const P &p1, const P &p2) {
    return norm((p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z));
}

inline double distance(const geometry_msgs::msg::PoseStamped &a,
                       const geometry_msgs::msg::PoseStamped &b) {
    return distance(a.pose.position, b.pose.position);
}

template <typename P1, typename P2> P1 subtractPoints(const P1 &p1, const P2 &p2) {
    P1 new_p;
    new_p.x = p1.x - p2.x;
    new_p.y = p1.y - p2.y;
    new_p.z = p1.z - p2.z;
    return new_p;
}

PathManager::PathManager()
    : Node("path_manager"),
      tf_listener_(nullptr),
      canceling_(false),
      explorer_manager_(nullptr),
      goal_init_(false),
      adjustment_margin_(0.5),
      setpoint_acceptance_radius_(0.5),
      goal_acceptance_radius_(2.0),
      obstacle_dist_threshold_(2.0),
      percent_above_(-1.0),
      percent_above_threshold_(0.01),
      mavros_map_frame_("map"),
      adjust_goal_altitude_(false),
      adjust_setpoint_(false),
      adjust_altitude_volume_(false),
      do_slam_(true),
      target_altitude_(3.0),
      planning_horizon_(6.0),
      velocity_setpoint_speed_(0.5),
      min_altitude_(3.0),
      max_altitude_(6.0),
      explorable_goals_(true) {}

void PathManager::initialize() {
    explorer_manager_ = std::make_shared<explorer::Explorer>(this);
    explorer_manager_->initialize();

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
    planning_horizon_ -= 1; // Subtract 1 for a safety margin to ensure the segmented goals are
                            // fully inside the regional cloud
    this->get_parameter("velocity_setpoint_speed", velocity_setpoint_speed_);
    this->get_parameter("explorable_goals", explorable_goals_);

    // Subscribers
    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", rclcpp::SensorDataQoS(),
        std::bind(&PathManager::positionCallback, this, _1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/search_node/trajectory_position", 10, std::bind(&PathManager::handlePath, this, _1));
    // percent_above_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    //     "/pcl_analysis/percent_above", 10, std::bind(&PathManager::percentAboveCallback, this, _1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_map", 10, std::bind(&PathManager::pointCloudCallback, this, _1));
    raw_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        raw_goal_topic, 10, std::bind(&PathManager::rawGoalCallback, this, _1));
    clicked_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathManager::rawGoalCallback, this, _1));
    cancel_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/path_manager/cancel", 10, std::bind(&PathManager::cancelCallback, this, _1));

    // Publishers
    mavros_setpoint_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
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

    checkAndUpdateGoal();

    checkAndUpdateSetpoint();
}

void PathManager::cancelCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != mavros_map_frame_) {
        canceling_ = false;
        RCLCPP_WARN(this->get_logger(), "PathManager un-canceling.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "PathManager received cancel request");
    home_pos_ = *msg;
    sub_goals_.clear();
    path_.clear();
    current_goal_ = geometry_msgs::msg::PoseStamped();
    current_setpoint_ = geometry_msgs::msg::PoseStamped();
    actual_path_.poses.clear();
    actual_path_pub_->publish(actual_path_);
    canceling_ = true;
    RCLCPP_INFO(this->get_logger(), "PathManager canceled all goals and path.");
}

void PathManager::checkAndUpdateGoal() {
    // Publish next goal when we have reached current one
    if (!sub_goals_.empty() && isCloseToGoal()) {

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
        updateGoal(current_goal_);
    }
}

void PathManager::checkAndUpdateSetpoint() {

    // No path, nothing to do
    if (path_.size() < 1)
        return;

    // Check if we are close enough to current setpoint to get the next part of the
    // path. Do as a while loop so that we publish the furthest setpoint that is still within the
    // acceptance radius
    while (isCloseToSetpoint() || isCloserThanSetpoint()) {
        // Get furthest ahead setpoint and set as current setpoint
        path_.erase(path_.begin());
        if (path_.size() == 0) {
            RCLCPP_INFO(this->get_logger(), "Path complete");
            return;
        }

        current_setpoint_ = path_[0];
        updateSetpoint(true);
    }

    // If last published setpoint is older than 1 second, republish it. This is to get the drone
    // "un-stuck" if it goes off path
    if (last_published_setpoint_time_.seconds() > 0 &&
        this->get_clock()->now() - last_published_setpoint_time_ > 1s) {
        RCLCPP_INFO(this->get_logger(), "Path manager republishing last setpoint");
        updateSetpoint(false);
    }
}

bool PathManager::isCloserThanSetpoint() {
    if (path_.size() < 1)
        return false;

    auto d = subtractPoints(path_.back().pose.position, current_setpoint_.pose.position);
    auto r = subtractPoints(current_pos_.pose.position, current_setpoint_.pose.position);

    bool is_closer = dotProduct(r, d) > dotProduct(d, d);
    return is_closer;
}

void PathManager::updateGoal(geometry_msgs::msg::PoseStamped goal) {
    goal.header.frame_id = mavros_map_frame_;

    if (adjust_altitude_volume_) {
        double altitude;
        if (!adjustAltitudeVolume(current_goal_.pose.position, altitude, min_altitude_,
                                    max_altitude_)) {
            RCLCPP_ERROR(this->get_logger(),
                            "Failed to adjust altitude volume, not publishing goal");
            return;
        }
        current_goal_.pose.position.z =
            altitude; // Adjust the goal altitude based on the volume
        RCLCPP_INFO(this->get_logger(), "Setting goal altitude to: %f", altitude);
        if (!do_slam_) {
            // Publish goal directly as MAVROS setpoint if not using path planner
            publishGoalAsMavrosSetpoint(current_goal_);
        } else {
            if (explorable_goals_) {
                RCLCPP_INFO(this->get_logger(),
                                "Path manager requesting explorable goal for path planner");
                geometry_msgs::msg::Pose goal_request = explorer_manager_->makeNewGoal(current_goal_.pose, min_altitude_, max_altitude_);
                current_goal_.pose = goal_request;
            }
            // Publish to "/goal" topic which requests path from path planner, which is
            // received in handlePath
            publishGoal(current_goal_);
        }
    } else {
        if (!do_slam_) {
            publishGoalAsMavrosSetpoint(goal);
        } else {
            if (explorable_goals_) {
                RCLCPP_INFO(this->get_logger(),
                                "Path manager requesting explorable goal for path planner");
                geometry_msgs::msg::Pose goal_request = explorer_manager_->makeNewGoal(current_goal_.pose, min_altitude_, max_altitude_);
                current_goal_.pose = goal_request;
            }
            publishGoal(goal);
        }
    }
}

void PathManager::publishGoal(geometry_msgs::msg::PoseStamped &goal) {
    // Get orientation (just for visualization)
    auto direction_vec = subtractPoints(goal.pose.position, current_pos_.pose.position);
    double yaw_target = atan2(direction_vec.y, direction_vec.x);

    tf2::Quaternion goal_q;
    goal_q.setRPY(0.0, 0.0, yaw_target);
    tf2::convert(goal_q, goal.pose.orientation);

    // Publish goal
    goal_pub_->publish(goal);
}

void PathManager::publishGoalAsMavrosSetpoint(const geometry_msgs::msg::PoseStamped &goal) {
    // Get goal data
    auto direction_vec = subtractPoints(goal.pose.position, current_pos_.pose.position);
    double yaw_target = atan2(direction_vec.y, direction_vec.x);

    // Publish goal
    mavros_msgs::msg::PositionTarget setpoint;
    setpoint.header.frame_id = mavros_map_frame_;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.position = goal.pose.position;
    setpoint.yaw = yaw_target;
    setpoint.coordinate_frame = setpoint.FRAME_LOCAL_NED;
    setpoint.type_mask |= setpoint.IGNORE_AFX | setpoint.IGNORE_AFY | setpoint.IGNORE_AFZ |
                          setpoint.IGNORE_VX | setpoint.IGNORE_VY | setpoint.IGNORE_VZ |
                          setpoint.IGNORE_YAW_RATE;

    RCLCPP_INFO(this->get_logger(), "Path manager publishing MAVROS goal: [%f, %f, %f]",
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
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

bool PathManager::adjustAltitudeVolume(const geometry_msgs::msg::Point &map_position,
                                       double &target_altitude, double &min_altitude,
                                       double &max_altitude) {
    std::shared_ptr<rclcpp::Node> get_elevation_node =
        rclcpp::Node::make_shared("get_elevation_node");
    auto get_elevation_client = get_elevation_node->create_client<messages_88::srv::GetMapData>(
        "/task_manager/get_map_data");
    auto elevation_req = std::make_shared<messages_88::srv::GetMapData::Request>();
    elevation_req->map_position = map_position;
    elevation_req->adjust_params = true;
    elevation_req->width = 8;
    elevation_req->height = 8;

    auto result = get_elevation_client->async_send_request(elevation_req);
    if (rclcpp::spin_until_future_complete(get_elevation_node, result, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (response->success) {
            target_altitude = response->target_altitude;
            min_altitude = response->min_altitude;
            max_altitude = response->max_altitude;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get elevation result");
            return false;
        }

    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call elevation service");
        return false;
    }
    return true;
}

void PathManager::pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg) {
    pcl::fromROSMsg(msg, *cloud_map_);

    if (goal_init_ && adjust_goal_altitude_) {
        if (adjustGoalAltitude(current_goal_)) {
            updateGoal(current_goal_);
        }
    }
}

void PathManager::rawGoalCallback(const geometry_msgs::msg::PoseStamped &msg) {

    RCLCPP_INFO(this->get_logger(), "Received raw goal [%f, %f, %f]", msg.pose.position.x,
                msg.pose.position.y, msg.pose.position.z);

    if (canceling_) {
        // Only allow raw goal if it is home pos
        if (!(msg.pose.position.x == home_pos_.pose.position.x &&
              msg.pose.position.y == home_pos_.pose.position.y &&
              msg.pose.position.z == home_pos_.pose.position.z)) {
            RCLCPP_WARN(this->get_logger(), "PathManager is canceling, ignoring raw goal other than home.");
            return;
        }
    }
    
    sub_goals_ = segmentGoal(msg);
    current_goal_ = sub_goals_.at(0);

    if (adjust_goal_altitude_) {
        adjustGoalAltitude(current_goal_);
    }

    updateGoal(current_goal_);
}

std::vector<geometry_msgs::msg::PoseStamped>
PathManager::segmentGoal(geometry_msgs::msg::PoseStamped goal) {

    double distance = decco_utilities::distance_xy(current_pos_.pose.position, goal.pose.position);

    int num_segments = std::ceil(distance / planning_horizon_);

    std::vector<geometry_msgs::msg::PoseStamped> sub_goals;

    // Calculate and store the interpolated points
    for (int i = 1; i <= num_segments; ++i) {
        geometry_msgs::msg::PoseStamped Pi;
        double t = static_cast<double>(i) / static_cast<double>(num_segments);
        Pi.pose.position.x = current_pos_.pose.position.x +
                             t * (goal.pose.position.x - current_pos_.pose.position.x);
        Pi.pose.position.y = current_pos_.pose.position.y +
                             t * (goal.pose.position.y - current_pos_.pose.position.y);
        Pi.pose.position.z = current_pos_.pose.position.z +
                             t * (goal.pose.position.z - current_pos_.pose.position.z);

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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Path Manager: Acceptable goal not found");
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
        findClosestPointInCloud(cloud_map_, goal.pose.position, closest_point,
                                closest_point_distance);
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

void PathManager::handlePath(const nav_msgs::msg::Path &path) {

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
}

void PathManager::updateSetpoint(bool use_velocity) {

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
    // Create vector pointing from current setpoint to point ahead on path, using 5 points ahead as
    // rough estimation
    // TODO play with this number
    auto vec =
        subtractPoints(path_[viable_pts_ahead].pose.position, current_setpoint_.pose.position);

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
        } else {
            unit_vec.x = 0.0;
            unit_vec.y = 0.0;
            unit_vec.z = 0.0;
        }

        // Determine absolute drone speed, initially set to velocity setpoint speed param
        double speed = velocity_setpoint_speed_;

        // Calculate distance to end of path. As we approach path end, reduce
        // speed using v^2 = u^2 + 2*a*s, with safe estimated acceleration of 1 m/s/s
        // Threshold to begin slowing down
        // distance = (speed^2)/(2*a)
        // TODO more advanced jerk and acceleration limiting based on drone dynamics
        double dist_to_end = distance(current_setpoint_, path_.back());
        double acceleration = 0.5;

        double dist_threshold = speed * speed / (2.0 * acceleration);
        if (dist_to_end < dist_threshold) {
            // Calculate new speed
            speed *= dist_to_end / dist_threshold;
        }

        // If within acceptance radius, set speed to 0.
        if (dist_to_end < setpoint_acceptance_radius_) {
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

    findClosestPointInCloud(cloud_map_, current_setpoint_.pose.position, closest_point,
                            closest_point_distance);

    // Check if setpoint is within our obstacle distance threshold to its closest point in the PCL.
    // If so, adjust setpoint so that it is outside threshold, but as near as possible to original
    // setpoint
    if (closest_point_distance < obstacle_dist_threshold_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Setpoint inside obstacle distance threshold, adjusting setpoint");
        float scale_factor = obstacle_dist_threshold_ / closest_point_distance;

        float dist_x = current_setpoint_.pose.position.x - closest_point.x;
        float dist_y = current_setpoint_.pose.position.y - closest_point.y;
        float dist_z = current_setpoint_.pose.position.z - closest_point.z;

        current_setpoint_.pose.position.x = closest_point.x + dist_x * scale_factor;
        current_setpoint_.pose.position.y = closest_point.y + dist_y * scale_factor;
        current_setpoint_.pose.position.z = closest_point.z + dist_z * scale_factor;
    }
}

void PathManager::findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          geometry_msgs::msg::Point point_in,
                                          pcl::PointXYZ &closest_point,
                                          float &closest_point_distance) {

    closest_point_distance = INFINITY;
    for (auto &point : *cloud) {
        float dist_x = point_in.x - point.x;
        float dist_y = point_in.y - point.y;
        float dist_z = point_in.z - point.z;

        float total_dist = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);

        if (total_dist < closest_point_distance) {
            closest_point_distance = total_dist;
            closest_point = point;
        }
    }
}

} // namespace path_manager
