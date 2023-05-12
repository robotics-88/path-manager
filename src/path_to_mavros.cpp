/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_to_mavros/path_to_mavros.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace path_to_mavros
{
PathToMavros::PathToMavros(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
{
}

}