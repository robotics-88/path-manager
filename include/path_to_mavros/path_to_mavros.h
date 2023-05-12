/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef PATH_TO_MAVROS_H_
#define PATH_TO_MAVROS_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

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

        double default_speed_;
        bool position_received_;
        geometry_msgs::PoseStamped last_pos_;
        geometry_msgs::PoseStamped curr_pos_;

        nav_msgs::Path actual_path_;
        geometry_msgs::PoseStamped current_goal_;
        double curr_yaw_;
        geometry_msgs::PoseStamped last_goal_;
        std::vector<geometry_msgs::PoseStamped> path_;

        std::string path_topic_;
        std::string frame_id_;

        ros::Subscriber position_sub_;
        ros::Subscriber path_sub_;

        ros::Publisher mavros_waypoint_publisher_;
        ros::Publisher actual_path_pub_;

        void positionCallback(const geometry_msgs::PoseStamped& msg);
        void setCurrentPath(const nav_msgs::Path::ConstPtr &path);
        void setPose(const geometry_msgs::PoseStamped& new_pose);
        void cmdLoopCallback(const ros::TimerEvent& event);
        void publishSetpoint();
        bool isCloseToGoal();
};

}

#endif