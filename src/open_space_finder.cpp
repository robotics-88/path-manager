/* 
© 2025 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "explorer/open_space_finder.h"

#include <pcl/common/common.h>

using PointT = pcl::PointXYZ;
using std::placeholders::_1;

namespace open_space_finder
{

OpenSpaceFinder::OpenSpaceFinder() 
    : search_radius_(5.0)
    , voxel_size_(0.1)
    , min_distance_threshold_(0.0)
    , max_open_centers_(10)
    , z_limit_(2.0)
{
}

void OpenSpaceFinder::initialize(const float search_radius, const float voxel_size, 
                                 const float min_distance_threshold, const int max_open_centers, const float z_limit)
{
    search_radius_ = search_radius;
    voxel_size_ = voxel_size;
    min_distance_threshold_ = min_distance_threshold;
    max_open_centers_ = max_open_centers;
    z_limit_ = z_limit;
}

void OpenSpaceFinder::setCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_cloud_ = msg;
}

void OpenSpaceFinder::setPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_pose_ = msg;
}

std::vector<decision_maker::NavOption> OpenSpaceFinder::searchFrom(const geometry_msgs::msg::Pose &pose_msg, const float &min_altitude, const float &max_altitude)
{
    if (!latest_cloud_)
    {
        std::cout << "No point cloud received yet." << std::endl;
        // RCLCPP_WARN(node_->get_logger(), "No point cloud received yet.");
        return {};
    }
    if (latest_cloud_->data.empty())
    {   
        std::cout << "Point cloud is empty." << std::endl;
        std::vector<decision_maker::NavOption> options;
        // Convert input to decision_maker::NavOption
        decision_maker::NavOption option;
        option.location.x = pose_msg.position.x; // Use the pose as the center
        option.location.y = pose_msg.position.y; // Use the pose as the center
        option.location.z = pose_msg.position.z; // Use the pose as the center
        option.size = 1.0; // Placeholder for size, can be calculated based on voxel later
        option.safety = 10.0; // Arbitrary but large, it doesn't matter
        options.push_back(option);
        return options;
    }
    std::cout << "Searching for open regions in free space with altitudes " << min_altitude 
              << " to " << max_altitude << " from pose: "
              << pose_msg.position.x << ", " << pose_msg.position.y << ", " << pose_msg.position.z << std::endl;
    auto open_regions = findOpenRegionsInFreeSpace(latest_cloud_, pose_msg, min_altitude, max_altitude);
    std::cout << "Found " << open_regions.size() << " open regions." << std::endl;
    return open_regions;
}

std::vector<decision_maker::NavOption> OpenSpaceFinder::findOpenRegionsInFreeSpace(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, const geometry_msgs::msg::Pose& start_pose, const float &min_altitude, const float &max_altitude)
{
    std::vector<decision_maker::NavOption> options;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // Get bounds of the point cloud
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);


    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    Eigen::Vector3f origin(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    int steps = std::ceil((2 * search_radius_) / voxel_size_);
    std::vector<std::pair<Eigen::Vector3f, float>> free_voxels;

    float z_min = min_altitude;
    float z_max = max_altitude;

    Eigen::Quaternionf q(latest_pose_->pose.orientation.w,
        latest_pose_->pose.orientation.x,
        latest_pose_->pose.orientation.y,
        latest_pose_->pose.orientation.z);
    Eigen::Vector3f forward = q * Eigen::Vector3f::UnitX();

    for (int x = -steps / 2; x <= steps / 2; ++x)
    {
        for (int y = -steps / 2; y <= steps / 2; ++y)
        {
            for (int z = -steps / 2; z <= steps / 2; ++z)
            {
                Eigen::Vector3f voxel = origin + Eigen::Vector3f(x, y, z) * voxel_size_;
                if ((voxel - origin).norm() > search_radius_) continue;
                if (voxel.z() < z_min || voxel.z() > z_max) continue;

                if (voxel.x() < min_pt.x || voxel.x() > max_pt.x) continue;
                if (voxel.y() < min_pt.y || voxel.y() > max_pt.y) continue;
                if (voxel.z() < min_pt.z || voxel.z() > max_pt.z) continue;

                Eigen::Vector3f drone_pos(
                    latest_pose_->pose.position.x,
                    latest_pose_->pose.position.y,
                    latest_pose_->pose.position.z);
                
                Eigen::Vector3f to_target = (origin - drone_pos).normalized();  // Direction from drone to search point
                Eigen::Vector3f to_voxel = (voxel - drone_pos).normalized();   // Direction from drone to voxel
                
                if (to_voxel.dot(to_target) < 0.1f) continue;  // Only accept voxels within X degrees cone toward the target
                

                PointT query;
                query.x = voxel.x();
                query.y = voxel.y();
                query.z = voxel.z();

                std::vector<int> indices(1);
                std::vector<float> sqr_dists(1);
                if (kdtree.nearestKSearch(query, 1, indices, sqr_dists) > 0)
                {
                    float dist = std::sqrt(sqr_dists[0]);
                    if (dist >= min_distance_threshold_)
                    {
                        free_voxels.emplace_back(voxel, dist);
                    }
                }
            }
        }
    }

    std::sort(free_voxels.begin(), free_voxels.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Greedy max-min selection
    std::vector<Eigen::Vector3f> open_centers;
    std::vector<float> distances;
    if (!free_voxels.empty())
        open_centers.push_back(free_voxels.front().first);

    while (open_centers.size() < static_cast<size_t>(max_open_centers_) && open_centers.size() < free_voxels.size())
    {
        float best_score = -1.0f;
        size_t best_index = 0;

        for (size_t i = 0; i < free_voxels.size(); ++i)
        {
            const Eigen::Vector3f& candidate = free_voxels[i].first;

            float min_dist = std::numeric_limits<float>::max();
            for (const auto& selected : open_centers)
            {
                float dist = (candidate - selected).norm();
                if (dist < min_dist) min_dist = dist;
            }

            if (min_dist > best_score)
            {
                best_score = min_dist;
                best_index = i;
            }
        }

        open_centers.push_back(free_voxels[best_index].first);
        distances.push_back(free_voxels[best_index].second);
    }
    saveMarkers(open_centers);

    // Convert open centers to decision_maker::NavOption
    for (int ii = 0; ii < open_centers.size(); ++ii)
    {
        const Eigen::Vector3f& center = open_centers[ii];
        decision_maker::NavOption option;
        option.location.x = center.x();
        option.location.y = center.y();
        option.location.z = center.z();
        option.size = 1.0; // Placeholder for size, can be calculated based on voxel later
        option.min_distance = (open_centers[ii] - Eigen::Vector3f(start_pose.position.x, start_pose.position.y, start_pose.position.z)).norm();
        option.safety = distances[ii];
        options.push_back(option);
    }

    return options;
}

void OpenSpaceFinder::saveMarkers(const std::vector<Eigen::Vector3f>& centers)
{
    latest_markers_.ns = "open_space";
    latest_markers_.id = 0;
    latest_markers_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    latest_markers_.action = visualization_msgs::msg::Marker::ADD;
    latest_markers_.scale.x = latest_markers_.scale.y = latest_markers_.scale.z = 0.3;
    latest_markers_.color.a = 1.0;
    latest_markers_.color.r = 0.0;
    latest_markers_.color.g = 1.0;
    latest_markers_.color.b = 0.0;

    for (const auto& c : centers)
    {
        geometry_msgs::msg::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = c.z();
        latest_markers_.points.push_back(p);
    }
}
}
