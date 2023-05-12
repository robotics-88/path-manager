/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_to_mavros/path_to_mavros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_to_mavros");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  path_to_mavros::PathToMavros pathToMavros(node);

  ros::spin();

  return 0;
}