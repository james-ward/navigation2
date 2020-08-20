// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#include "smac_planner/node_se2.hpp"

namespace smac_planner
{

// defining static member for all instance to share
MotionTable NodeSE2::_motion_model;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/node821.html
void MotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & angle_quantization_in)
{
  size_x = size_x_in;
  angle_quantization = angle_quantization_in;
  projections.clear();
  projections.reserve(3);//TODO
  projections.emplace_back(1.0, 0.0, 0.0);  // Forward   
  projections.emplace_back(0.0, 1.0, 0.0);  // Forward   
  projections.emplace_back(0.0, -1.0, 0.0);  // Forward   
  // projections.emplace_back(???, ???, angle_quantization_in);  // Left by 1 angular bin
  // projections.emplace_back(???, -???, -angle_quantization_in);  // Right by 1 angular bin
}

// http://planning.cs.uiuc.edu/node822.html
void MotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & angle_quantization_in) //TODO
{
  size_x = size_x_in;
  angle_quantization = angle_quantization_in;
  projections.clear();
  projections.reserve(6);
}

// http://planning.cs.uiuc.edu/node823.html
void MotionTable::initBalkcomMason(
  unsigned int & size_x_in,
  unsigned int & angle_quantization_in) //TODO
{
  size_x = size_x_in;
  angle_quantization = angle_quantization_in;
  projections.clear();
  projections.reserve(6);
}

Poses MotionTable::getProjections(NodeSE2 * & node)
{
  Poses projection_list;
  for (uint i = 0; i != projections.size(); i++) {
    projection_list.push_back(getProjection(node, i));
  }

  return projection_list;
}

Pose MotionTable::getProjection(NodeSE2 * & node, const unsigned int & motion_index)
{
  return node->pose + projections[motion_index];
}

NodeSE2::NodeSE2(unsigned char & cost_in, const unsigned int index)
: parent(nullptr),
  _cell_cost(static_cast<float>(cost_in)),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false)
{
}

NodeSE2::~NodeSE2()
{
  parent = nullptr;
}

void NodeSE2::reset(const unsigned char & cost, const unsigned int index)
{
  parent = nullptr;
  _cell_cost = static_cast<float>(cost);
  _accumulated_cost = std::numeric_limits<float>::max();
  _index = index;
  _was_visited = false;
  _is_queued = false;
}

bool NodeSE2::isNodeValid(const bool & traverse_unknown) {
  // NOTE(stevemacenski): Right now, we do not check if the node has wrapped around
  // the regular grid (e.g. your node is on the edge of the costmap and i+1
  // goes to the other side). This check would add compute time and my assertion is
  // that if you do wrap around, the heuristic will be so high it'll be added far
  // in the queue that it will never be called if a valid path exists.
  // This is intentionally un-included to increase speed, but be aware. If this causes
  // trouble, please file a ticket and we can address it then.

  // occupied node
  auto & cost = this->getCost();
  if (cost == OCCUPIED || cost == INSCRIBED) {
    return false;
  }

  // unknown node
  if (cost == UNKNOWN && ! traverse_unknown) {
    return false;
  }

  return true;
}

float NodeSE2::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coordinates)
{
  return hypotf(
    goal_coordinates.x - node_coords.x,
    goal_coordinates.y - node_coords.y);
}

void NodeSE2::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & angle_quantization)
{
  // find the motion model selected
  switch(motion_model) {
    case MotionModel::DUBIN:
      _motion_model.initDubin(size_x, angle_quantization);
      break;
    case MotionModel::REEDS_SHEPP:
      _motion_model.initReedsShepp(size_x, angle_quantization);
      break;
    case MotionModel::BALKCOM_MASON:
      _motion_model.initBalkcomMason(size_x, angle_quantization);
      break;
    default:
      throw std::runtime_error("Invalid motion model for SE2 node. Please select between"
        " Dubin (Ackermann forward only),"
        " Reeds-Shepp (Ackermann forward and back),"
        " or Balkcom-Mason (Differential drive and omnidirectional) models.");
  }
}

void NodeSE2::getNeighbors(
  NodePtr & node,
  std::function<bool(const unsigned int&, smac_planner::NodeSE2*&)> & validity_checker,
  NodeVector & neighbors)
{
  int index;
  NodePtr neighbor = nullptr;
  Poses projections = _motion_model.getProjections(node);

  for(unsigned int i = 0; i != projections.size(); ++i) {
    index = NodeSE2::getIndex(
      projections[i]._x, projections[i]._y, projections[i]._theta,
      _motion_model.size_x, _motion_model.angle_quantization);
    if (validity_checker(index, neighbor))
    {
      neighbor->setPose(projections[i]);
      neighbors.push_back(neighbor);
    }
  }
}

}  // namespace smac_planner
