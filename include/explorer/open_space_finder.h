/* 
© 2025 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <Eigen/Dense>

#include "explorer/decision_maker.h"

namespace open_space_finder
{
/**
 * @class OpenSpaceFinder
 * @brief Searches pointcloud for open regions in free space
 */

class OpenSpaceFinder
{
public:
    OpenSpaceFinder();
    
    void initialize(const float search_radius, const float voxel_size, 
        const float min_distance_threshold, const int max_open_centers, const float z_limit);
    void setCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void setPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    visualization_msgs::msg::Marker getMarkers() {
        return latest_markers_;
    }
    std::vector<decision_maker::NavOption> searchFrom(const geometry_msgs::msg::Pose &pose_msg, const float &min_altitude, const float &max_altitude);

private:

    std::vector<decision_maker::NavOption> findOpenRegionsInFreeSpace(
        const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
        const geometry_msgs::msg::Pose& start_pose, const float &min_altitude, const float &max_altitude);

    void saveMarkers(const std::vector<Eigen::Vector3f>& centers);

    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
    visualization_msgs::msg::Marker latest_markers_;

    // Parameters
    float search_radius_;
    float voxel_size_;
    float min_distance_threshold_;
    int max_open_centers_;
    float z_limit_;
    std::string pointcloud_topic_;
};

}