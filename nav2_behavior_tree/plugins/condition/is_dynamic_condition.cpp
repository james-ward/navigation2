// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__IS_DYNAMIC_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__IS_DYNAMIC_CONDITION_HPP_

#include <string>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

class IsIsDynamicCondition : public BT::ConditionNode
{
public:
  IsIsDynamicCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    is_dynamic_(false),
    min_count_(3)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("min_count", min_count_);
    obj_sub_ = node_->create_subscription<std_msgs::msg::Int32>("num_dynamic_obstacles",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&IsIsDynamicCondition::onObstacleReceieved, this, std::placeholders::_1));
  }

  IsIsDynamicCondition() = delete;

  void onObstacleReceieved(const typename std_msgs::msg::Int32::SharedPtr msg)
  {
    is_dynamic_ = msg->data > min_count_ ? true : false;
  }

  BT::NodeStatus tick() override
  {
    if (is_dynamic_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_count", 3, "Minimum count of "
        "moving obstacles to be considered a dynamic environment.")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> is_dynamic_;
  int min_count_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr obj_sub_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsIsDynamicCondition>("IsDynamic");
}

#endif  // NAV2_BEHAVIOR_TREE__IS_DYNAMIC_CONDITION_HPP_
