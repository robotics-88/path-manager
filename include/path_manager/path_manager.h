/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#ifndef PATH_MANAGER_H_
#define PATH_MANAGER_H_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "messages_88/srv/get_map_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace path_manager {
/**
 * @class PathManager
 * @brief The PathManager class handles things related to the path of the drone
 */
class PathManager : public rclcpp::Node {
  public:
    PathManager();
    ~PathManager();

  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string mavros_map_frame_;

    double setpoint_acceptance_radius_;
    double goal_acceptance_radius_;
    double obstacle_dist_threshold_;
    bool adjust_goal_altitude_;
    bool adjust_setpoint_;
    bool adjust_altitude_volume_;
    bool do_slam_;
    geometry_msgs::msg::PoseStamped current_pos_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_;

    nav_msgs::msg::Path actual_path_;
    geometry_msgs::msg::PoseStamped current_setpoint_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    std::vector<geometry_msgs::msg::PoseStamped> sub_goals_;

    rclcpp::Time last_published_setpoint_time_;

    geometry_msgs::msg::PoseStamped current_goal_;
    bool goal_init_;
    float adjustment_margin_;

    double yaw_target_;
    double target_altitude_;
    double planning_horizon_;
    double velocity_setpoint_speed_;

    std::string frame_id_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_goal_sub_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavros_setpoint_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    rclcpp::Client<messages_88::srv::GetMapData>::SharedPtr get_elevation_client_;

    void checkAndUpdateGoal();
    void checkAndUpdateSetpoint();

    void positionCallback(const geometry_msgs::msg::PoseStamped &msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg);
    void rawGoalCallback(const geometry_msgs::msg::PoseStamped &msg);

    void handlePath(const nav_msgs::msg::Path &path);
    void updateSetpoint(bool use_velocity);
    bool isCloseToSetpoint();
    void adjustSetpoint();
    void findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 geometry_msgs::msg::Point point_in, pcl::PointXYZ &closest_point,
                                 float &closest_point_distance);
    std::vector<geometry_msgs::msg::PoseStamped> segmentGoal(geometry_msgs::msg::PoseStamped goal);

    bool isCloseToGoal();
    bool isCloserThanSetpoint();
    bool adjustGoalAltitude(geometry_msgs::msg::PoseStamped goal);
    void updateGoal(geometry_msgs::msg::PoseStamped goal);
    void publishGoal(geometry_msgs::msg::PoseStamped &goal);
    void publishGoalAsMavrosSetpoint(const geometry_msgs::msg::PoseStamped &goal);
    void requestPath(const geometry_msgs::msg::PoseStamped goal);
    void adjustAltitudeVolume(const geometry_msgs::msg::Point &map_position,
                              std::function<void(bool, double)> callback);
};

} // namespace path_manager

#endif