/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef PATH_TO_MAVROS_H_
#define PATH_TO_MAVROS_H_

#include <ros/ros.h>

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

};

}

#endif