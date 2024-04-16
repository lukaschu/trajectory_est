#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_est/traj_estimator_node.hpp"

int main(int argc, char ** argv)
{
  // Ros functionality
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryEstimator>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}