/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "path_manager/path_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_manager");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  path_manager::PathManager PathManager(node);

  ros::spin();

  return 0;
}