// Copyright (c) 2020, OUXT-Polaris
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

#ifndef ROBOTX_BT_PLANNER__BT_PLANNER_COMPONENT_HPP_
#define ROBOTX_BT_PLANNER__BT_PLANNER_COMPONENT_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "robotx_bt_planner/visibility.h"

namespace robotx_bt_planner
{
class BTPlannerComponent : public rclcpp::Node
{
public:
  ROBOTX_BT_PLANNER_PUBLIC
  explicit BTPlannerComponent(const rclcpp::NodeOptions & options);

private:
  void timerCallback();
  bool loadPlugin(std::string);
  bool loadTree(std::string);

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
  rclcpp::Node::SharedPtr client_node_;
};
}  // namespace robotx_bt_planner

#endif  // ROBOTX_BT_PLANNER__BT_PLANNER_COMPONENT_HPP_
