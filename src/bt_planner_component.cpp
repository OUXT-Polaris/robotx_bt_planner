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

#include <fstream>
#include <memory>
#include <string>

#include "robotx_bt_planner/bt_planner_component.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robotx_bt_planner
{
BTPlannerComponent::BTPlannerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("robotx_bt_planner", options)
{
  using std::chrono_literals::operator""ms;
  // 共有ライブラリからプラグインを読み込み
  loadPlugin("example_action");
  loadPlugin("go_action");
  std::cout << "REGISTERED PLUGINS : " << std::endl;
  std::cout << "=================================" << std::endl;
  for (auto builder : factory_.builders()) {
    std::cout << "  " << builder.first << std::endl;
  }
  std::cout << "=================================" << std::endl;


  // blackboard
  blackboard_ = BT::Blackboard::create();

  // client_node
  auto client_options = rclcpp::NodeOptions().arguments(
      {"--ros-args",
       "-r",std::string("__node:=") + get_name() + "_client_node",
       "--"}
  );
  client_node_ = std::make_shared<rclcpp::Node>("_", client_options);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);

  // server_timeout
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));


  loadTree("example");
  std::cout << "TREE CREATED" << std::endl;
  // Grootへ実行情報を送信する
  publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(tree_);
  std::cout << "PUBLISHER CREATED" << std::endl;
  timer_ = create_wall_timer(
    500ms, std::bind(&BTPlannerComponent::timerCallback, this));
  std::cout << "TIMER CREATED" << std::endl;


}
void BTPlannerComponent::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "tick %d",tree_.nodes.size());

  tree_.root_node->executeTick();
}
bool BTPlannerComponent::loadPlugin(std::string plugin_name)
{
  std::string plugin_filename =
    ament_index_cpp::get_package_share_directory("robotx_behavior_tree") +
    "/../../lib/lib" + plugin_name + ".so";
  factory_.registerFromPlugin(plugin_filename);
  return true;
}
bool BTPlannerComponent::loadTree(std::string xml_name)
{
  std::string bt_xml_filename = ament_index_cpp::get_package_share_directory("robotx_bt_planner") +
    "/behavior_trees/" + xml_name + ".xml";
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  tree_ = factory_.createTreeFromText(xml_string, blackboard_);
  return true;
}
}  // namespace robotx_bt_planner

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_bt_planner::BTPlannerComponent)
