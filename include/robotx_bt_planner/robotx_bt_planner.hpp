/**
 * Copyright 2019 OUXT-Polaris
 */

#ifndef ROBOTX_BT_PLANNER__ROBOTX_BT_PLANNER_HPP_
#define ROBOTX_BT_PLANNER__ROBOTX_BT_PLANNER_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>


namespace robotx_bt_planner
{
class BehaviorTreePlannerNode : public rclcpp::Node
{
public:
  BehaviorTreePlannerNode()
  : rclcpp::Node("robotx_bt_planner")
  {
    // 共有ライブラリからプラグインを読み込み
//    factory.registerFromPlugin(
//        "install/robotx_behavior_tree/lib/robotx_behavior_tree/"
//        "librobotx_behavior_tree_node.so");
    // Grootへ実行情報を送信する
    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
    timer = create_wall_timer(
      500ms, std::bind(&MinimalExecutorNode::timerCallback, this));
  }

private:
  void timerCallback();

  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
};
}  // namespace robotx_bt_planner
#endif  // ROBOTX_BT_PLANNER__ROBOTX_BT_PLANNER_HPP_
