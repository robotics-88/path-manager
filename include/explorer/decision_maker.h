/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/


#ifndef DECISION_MAKER_H_
#define DECISION_MAKER_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

// #include "TopographyDecision.h"

namespace decision_maker
{

struct NavOption {
  double min_distance; // Minimum distance to the goal
  double size;         // Size of the voxel
  geometry_msgs::msg::Point location; // Location of the option
  double safety;      // Safety score
  double cost;        // Overall cost for this option
};

enum Criteria {
  DIST,
  INFO,
  EFFICIENCY,
  SAFETY,
  ASH
};

/**
 * @class DecisionMaker
 * @brief Evaluates a list of open space points based on user-selected criteria
 */
class DecisionMaker
{
public:
  DecisionMaker();
  ~DecisionMaker();

  void init(const double res, const std::vector<std::string> criteria, const std::vector<double> weights);

  void evaluate(std::vector<NavOption> &option_list, const geometry_msgs::msg::Point prev_goal);

  void setMapResolution(const double res);

private:

  std::vector<double> optionScore(const NavOption& option);

  double weightedAverage(std::vector<double>& wt, std::vector<double>& scores);

  void normalizeSum(std::vector<double> &vec);

  double utilityFunction(decision_maker::Criteria criteria, const NavOption& option);

  void setCriteriaAndWeights(const std::vector<std::string> criteria, const std::vector<double> weights);

  // Support methods for complex evaluation functions
//   double valleyScore(const geometry_msgs::msg::Point &map_point);
//   bool initTopography();

  // Internal evaluation params
  double map_resolution_;
  int num_criteria_;
  int num_pairs_;
  std::vector<double> weights_;
  std::vector<decision_maker::Criteria> criteria_;
  geometry_msgs::msg::Point prev_goal_;
//   topography::TopographyDecision topography_dm_;
  bool has_ash_;
//   std::string dem_name_;
};
}

#endif
