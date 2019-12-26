// Copyright (c) 2019, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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
    using std::chrono_literals::operator""ms;
    // 共有ライブラリからプラグインを読み込み
    factory.registerFromPlugin(
      "install/robotx_behavior_tree/lib/robotx_behavior_tree/"
      "libexample_action.so");
    // Grootへ実行情報を送信する
    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
    timer = create_wall_timer(
      500ms, std::bind(&BehaviorTreePlannerNode::timerCallback, this));
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
