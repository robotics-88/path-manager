/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include <explorer/decision_maker.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <boost/math/special_functions/digamma.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace decision_maker
{
DecisionMaker::DecisionMaker()
  : has_ash_(false)
//   , dem_name_("ballard.tif")
{
}

DecisionMaker::~DecisionMaker() {

}

void DecisionMaker::init(const double res, const std::vector<std::string> criteria, const std::vector<double> weights) {
  setMapResolution(res);

  setCriteriaAndWeights(criteria, weights);
  normalizeSum(weights_);

  num_criteria_ = criteria_.size();
}

void DecisionMaker::evaluate(std::vector<NavOption> &option_list, const geometry_msgs::msg::Point prev_goal) {
  prev_goal_ = prev_goal;
  // set utilities of options
  std::vector<double> scores(num_criteria_,0);
  for (auto& option : option_list) {
    scores = optionScore(option);
    option.utility = weightedAverage(weights_, scores);
  }

  // sort options by utility, high score first
  std::sort(
      option_list.begin(), option_list.end(),
      [](const NavOption& no1, const NavOption& no2) { return no1.utility > no2.utility; });
}

double DecisionMaker::weightedAverage(std::vector<double>& wt, std::vector<double>& scores) {
  // Computes weighted average of utilities
  double global_score = 0.0;
  for (int ii = 0; ii < num_criteria_; ii++) {
    global_score += wt.at(ii) * scores.at(ii);
  }
  return global_score;
}

void DecisionMaker::setMapResolution(const double res) {
  map_resolution_ = res;
}

std::vector<double> DecisionMaker::optionScore(const NavOption& option)
{
  std::vector<double> util;

  for (int ii = 0; ii < num_criteria_; ii++) {
    util.push_back(utilityFunction(criteria_.at(ii), option));
  }

  return util;
}

double DecisionMaker::utilityFunction(decision_maker::Criteria criteria, const NavOption& option) {
  // All utility functions must be written so that larger scores are better, for consistency
  switch (criteria) {
    case DIST:
      return 1 / (option.min_distance * map_resolution_);
      break;
    case INFO:
      return (option.size * map_resolution_);
      break;
    case EFFICIENCY:
      return 1 / sqrt(pow(option.location.x - prev_goal_.x, 2) + pow(option.location.y - prev_goal_.y, 2));
      break;
    case SAFETY:
      return 1 / option.safety;
      break;
    case ASH:
    //   return valleyScore(option.midpoint);
      return 0.0; // Placeholder for ASH utility, needs reimplementation now that DEM moved
      break;
    default:
      return -1.0;
  }
}

void DecisionMaker::normalizeSum(std::vector<double> &vec) {
  double sm = 0;
  for (int ii = 0; ii < vec.size(); ii++) {
    sm += vec.at(ii);
  }
  double dbl_eq_threshold = .01;
  if (abs(sm - 1) < dbl_eq_threshold) {
    // RCLCPP_INFO(node_->get_logger(), "Normalize weights not needed, sum: %f", sm);
    return;
  }
  double mag_inv = 1 / sm;
  for (int ii = 0; ii < vec.size(); ii++) {
    vec.at(ii) = vec.at(ii) * mag_inv;
  }
}

void DecisionMaker::setCriteriaAndWeights(const std::vector<std::string> criteria, const std::vector<double> weights) {
  weights_.clear();
  criteria_.clear();
  for (int ii = 0; ii < criteria.size(); ii++) {
    if (criteria.at(ii) == "INFO") {
      criteria_.push_back(INFO);
    }
    else if (criteria.at(ii) == "DIST") {
      criteria_.push_back(DIST);
    }
    else if (criteria.at(ii) == "EFFICIENCY") {
      criteria_.push_back(EFFICIENCY);
    }
    else if (criteria.at(ii) == "SAFETY") {
      criteria_.push_back(SAFETY);
    }
    else if (criteria.at(ii) == "ASH") {
    //   has_ash_ = initTopography();
    //   if (!has_ash_) {
    //     continue;
    //   }
    //   criteria_.push_back(ASH);
    }
    else {
      continue;
    }
    weights_.push_back(weights.at(ii));
  }
}

// bool DecisionMaker::initTopography() {
//   // TODO replace with topography sent from Hello Decco (long term)
//   std::string tif_name = "/distal_ws/src/exploration/topog/" + dem_name_;
//   bool file_found = std::filesystem::exists(tif_name);
//   if (!file_found) {
//     RCLCPP_INFO(node_->get_logger(), "No topography file, proceeding without ASH criterion.");
//     return false;
//   }
  
//   while (!geo_client_->wait_for_service(5s)) {
//       if (!rclcpp::ok()) {
//         RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for geo client service. Proceeding without ASH criterion.");
//         return false;
//       }
//       RCLCPP_INFO(node_->get_logger(), "Geo client service not available, waiting again...");
//   }
//   return topography_dm_.init(tif_name);
// }

// double DecisionMaker::valleyScore(const geometry_msgs::msg::Point &map_point) {
//   // From topography matrix, determine how much of a valley
//   std::shared_ptr<rclcpp::Node> geo_node = rclcpp::Node::make_shared("geo_node");
//   auto geo_request = std::make_shared<messages_88::srv::Geopoint::Request>(); 
//   geo_request->slam_position = map_point;
//   auto geo_res = geo_client_->async_send_request(geo_request);
//   double valley;
//   try
//   {
//       auto response = geo_res.get();
//       bool valid_score = topography_dm_.valleyScore(response->utm_position.x, response->utm_position.y, valley);
//       if (valley != valley) {
//         // NaN: Give it a middle of the road score. TODO: something smarter later.
//         valley = 0.5;
//       }
//   }
//   catch (const std::exception &e)
//   {
//       RCLCPP_INFO(node_->get_logger(), "Geo service call failed.");
//       valley = 0.5;
//   }

//   return valley;
// }

}