/**
 * Copyright 2019 OUXT-Polaris
 */

#include <rclcpp/rclcpp.hpp>
#include <robotx_bt_planner/robotx_bt_planner.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotx_bt_planner::BehaviorTreePlannerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
