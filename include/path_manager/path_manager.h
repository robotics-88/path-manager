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
#include "mavros_msgs/msg/position_target.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "messages_88/srv/get_map_data.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"

namespace path_manager {
/**
 * @class PathManager
 * @brief The PathManager class handles things related to the path of the drone, both exploration goals and path management
 */
class PathManager : public rclcpp::Node
{
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
        float percent_above_threshold_;
        bool path_received_;
        bool adjust_goal_altitude_;
        bool adjust_setpoint_;
        bool adjust_altitude_volume_;
        bool do_slam_;
        geometry_msgs::msg::PoseStamped last_pos_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_;

        nav_msgs::msg::Path actual_path_;
        geometry_msgs::msg::PoseStamped next_setpoint_;
        geometry_msgs::msg::PoseStamped current_setpoint_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::vector<geometry_msgs::msg::PoseStamped> sub_goals_;

        rclcpp::Time last_published_setpoint_time_;

        geometry_msgs::msg::PoseStamped current_goal_;
        bool goal_active_;
        bool goal_init_;
        float adjustment_margin_;

        double yaw_target_;
        double target_altitude_;
        double planning_horizon_;
        double velocity_setpoint_speed_;

        float percent_above_;

        std::string frame_id_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr          percent_above_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   pointcloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_goal_sub_;

        
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr   mavros_setpoint_raw_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_setpoint_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    setpoint_viz_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                actual_path_pub_;

        void updateGoal();
        void updateSetpoint();

        void percentAboveCallback(const std_msgs::msg::Float32 &msg);
        void positionCallback(const geometry_msgs::msg::PoseStamped &msg);
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg);
        // void livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void rawGoalCallback(const geometry_msgs::msg::PoseStamped &msg);

        // pcl::PointCloud<pcl::PointXYZ> transformCloudToMapFrame(pcl::PointCloud<pcl::PointXYZ> cloud_in);
        void setCurrentPath(const nav_msgs::msg::Path &path);
        void publishSetpoint(bool use_velocity);
        bool isCloseToSetpoint();
        void adjustSetpoint();
        void findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::Point point_in, 
                                              pcl::PointXYZ &closest_point, float &closest_point_distance);
        bool isSafe(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const geometry_msgs::msg::Point point_in);
        std::vector<geometry_msgs::msg::PoseStamped> segmentGoal(geometry_msgs::msg::PoseStamped goal);

        bool isCloseToGoal();
        bool isCloserThanSetpoint();
        bool adjustGoalAltitude(geometry_msgs::msg::PoseStamped goal);
        void publishGoal(geometry_msgs::msg::PoseStamped goal);
        geometry_msgs::msg::PoseStamped requestGoal(const geometry_msgs::msg::PoseStamped goal);
        bool requestPath(const geometry_msgs::msg::PoseStamped goal);
        void adjustAltitudeVolume(const geometry_msgs::msg::Point &map_position, double &target_altitude);
};

}

#endif