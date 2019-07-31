// Copyright (c) 2018 Intel Corporation
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

#include <memory>

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_world_model/world_model.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

	auto nvFnNode = std::make_shared<nav2_navfn_planner::NavfnPlanner>();
  auto worldModelNode = std::make_shared<nav2_world_model::WorldModel>();   

  exec.add_node(worldModelNode->get_node_base_interface());
  exec.add_node(nvFnNode->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
