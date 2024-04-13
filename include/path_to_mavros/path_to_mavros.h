/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef PATH_TO_MAVROS_H_
#define PATH_TO_MAVROS_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_ros/point_cloud.h>


namespace path_to_mavros {
/**
 * @class PathToMavros
 * @brief The PathToMavros class converts ROS paths into individual MAVROS waypoints for execution
 */
class PathToMavros {

    public:
        PathToMavros(ros::NodeHandle& node);
        ~PathToMavros(){};

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string mavros_map_frame_;

        double acceptance_radius_;
        double obstacle_dist_threshold_;
        bool position_received_;
        bool goal_received_;
        geometry_msgs::PoseStamped last_pos_;
        pcl::PointCloud<pcl::PointXYZ> last_cloud_;

        nav_msgs::Path actual_path_;
        geometry_msgs::PoseStamped current_goal_;
        geometry_msgs::PoseStamped last_goal_;
        std::vector<geometry_msgs::PoseStamped> path_;

        double yaw_target_;

        std::string path_topic_;
        std::string frame_id_;

        ros::Subscriber position_sub_;
        ros::Subscriber path_sub_;
        ros::Subscriber pointcloud_sub_;

        ros::Publisher mavros_waypoint_publisher_;
        ros::Publisher actual_path_pub_;

        void positionCallback(const geometry_msgs::PoseStamped& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void setCurrentPath(const nav_msgs::Path::ConstPtr &path);
        void setPose(const geometry_msgs::PoseStamped& new_pose);
        void publishSetpoint();
        bool isCloseToGoal();
        void ensureSetpointSafety();
};

}

#endif