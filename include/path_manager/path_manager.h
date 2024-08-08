/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef PATH_MANAGER_H_
#define PATH_MANAGER_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_ros/point_cloud.h>


namespace path_manager {
/**
 * @class PathManager
 * @brief The PathManager class handles things related to the path of the drone, both exploration goals and path management
 */
class PathManager {

    public:
        PathManager(ros::NodeHandle& node);
        ~PathManager(){};

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string mavros_map_frame_;

        double acceptance_radius_;
        double obstacle_dist_threshold_;
        bool path_received_;
        bool adjust_goal_;
        bool adjust_setpoint_;
        geometry_msgs::PoseStamped last_pos_;
        pcl::PointCloud<pcl::PointXYZ> cloud_map_;

        nav_msgs::Path actual_path_;
        geometry_msgs::PoseStamped current_setpoint_;
        geometry_msgs::PoseStamped last_setpoint_;
        std::vector<geometry_msgs::PoseStamped> path_;

        geometry_msgs::PoseStamped current_goal_;
        bool goal_init_;
        float adjustment_margin_;

        double yaw_target_;

        std::string frame_id_;

        ros::Subscriber position_sub_;
        ros::Subscriber path_sub_;
        ros::Subscriber pointcloud_sub_;
        ros::Subscriber raw_goal_sub_;

        ros::Publisher mavros_setpoint_pub_;
        ros::Publisher actual_path_pub_;
        ros::Publisher goal_pub_;

        void positionCallback(const geometry_msgs::PoseStamped& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        // void livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void rawGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        pcl::PointCloud<pcl::PointXYZ> transformCloudToMapFrame(pcl::PointCloud<pcl::PointXYZ> cloud_in);
        void setCurrentPath(const nav_msgs::Path::ConstPtr &path);
        void publishSetpoint();
        bool isCloseToSetpoint();
        void adjustSetpoint();
        void findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Point point_in, 
                                              pcl::PointXYZ &closest_point, float &closest_point_distance);
        void adjustGoal(geometry_msgs::PoseStamped goal);
};

}

#endif