/**
 * Copyright 2019 OUXT-Polaris
 */

#include <robotx_bt_planner/robotx_bt_planner.hpp>

namespace robotx_bt_planner
{
void BehaviorTreePlannerNode::timerCallback()
{
  tree.root_node->executeTick();
  RCLCPP_INFO(this->get_logger(), "tick");
}
}  // namespace robotx_bt_planner
