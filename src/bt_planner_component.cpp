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

#include "robotx_bt_planner/bt_planner_component.hpp"
#include <memory>

namespace robotx_bt_planner
{

static const char * xml =
  R"(
    <root>
        <BehaviorTree>
            <Sequence>
                <ExampleAction/>
            </Sequence>
        </BehaviorTree>
    </root>
)";

BTPlannerComponent::BTPlannerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("robotx_bt_planner", options)
{
  using std::chrono_literals::operator""ms;
  // 共有ライブラリからプラグインを読み込み
  factory.registerFromPlugin(
    "/home/hans/ros2_eloquent_ws/install/robotx_behavior_tree/lib/libexample_action.so");
  std::cout << "REGISTERED PLUGINS : " << std::endl;
  std::cout << "=================================" << std::endl;
  for (auto builder : factory.builders()) {
    std::cout << "  " << builder.first << std::endl;
  }
  std::cout << "=================================" << std::endl;
  tree = factory.createTreeFromText(xml);
  // Grootへ実行情報を送信する
  publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
  timer = create_wall_timer(
    500ms, std::bind(&BTPlannerComponent::timerCallback, this));
}
void BTPlannerComponent::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "tick");
  tree.root_node->executeTick();
}
}  // namespace robotx_bt_planner

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_bt_planner::BTPlannerComponent)
