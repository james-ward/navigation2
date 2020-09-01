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

#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>

#include "smac_planner/a_star.hpp"

// TODO optimization: should we be constructing full SE2 graph at start?
    // or only construct as we get in contact to expand!
    // doesnt matter as much for 2D planner, but really will matter here
    // might speed things up a bit. Reserve graph (full, 20%, whatever) but dont fill in.

// TODO Pose, IndexPath, Coordinate homologation, Eigen::vector2d seems like alot of the same crap

namespace smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const float & min_turning_radius)
: _travel_cost_scale(0.0),
  _neutral_cost(0.0),
  _traverse_unknown(true),
  _max_iterations(0),
  _x_size(0),
  _y_size(0),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _graph(nullptr),
  _queue(nullptr),
  _motion_model(motion_model),
  _min_turning_radius(min_turning_radius)
{
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
  _graph.reset();
  _queue.reset();
  _start = nullptr;
  _goal = nullptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const float & travel_cost_scale,
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations)
{
  if (_graph) {
    _graph.reset();
  }

  if (_queue) {
    _queue.reset();
  }

  _graph = std::make_unique<Graph>();
  _queue = std::make_unique<NodeQueue>();
  _travel_cost_scale = travel_cost_scale;
  _neutral_cost = 253.0 * (1.0 - _travel_cost_scale);
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
}

template <>
void AStarAlgorithm<Node2D>::createGraph(
  const unsigned int & x,
  const unsigned int & y,
  const unsigned int & theta,
  unsigned char * & costs)
{
  if (theta != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 angle quantization.");
  }

  _dim3_size = theta;  // 2D search MUST be 2D, not 3D or SE2.

  if (getSizeX() != x || getSizeY() != y) {
    _x_size = x;
    _y_size = y;
    Node2D::initNeighborhood(_x_size, _motion_model);
    _graph->clear();
    _graph->reserve(x * y);
    for (unsigned int i = 0; i != x * y; i++) {
      _graph->emplace_back(costs[i], i);
    }
  } else {
    for (unsigned int i = 0; i != x * y; i++) {
      // Optimization: operator[] is used over at() for performance (no bound checking)
      _graph->operator[](i).reset(costs[i], i);
    }
  }
}

// Population order theta, Y, X to match getIndex expected order
template <>
void AStarAlgorithm<NodeSE2>::createGraph(
  const unsigned int & x,
  const unsigned int & y,
  const unsigned int & theta,
  unsigned char * & costs)
{
  _dim3_size = theta;
  unsigned int index;

  if (getSizeX() != x || getSizeY() != y) {
    _x_size = x;
    _y_size = y;
    NodeSE2::initMotionModel(_motion_model, _x_size, _dim3_size, _min_turning_radius);
    _graph->clear();
    _graph->reserve(x * y * _dim3_size);

    for (unsigned int i = 0; i != x; i++) {
      for (unsigned int j = 0; j != y; j++) {
        for (unsigned int k = 0; k != _dim3_size; k++) {
          index = NodeSE2::getIndex(i, j, k, _x_size, _dim3_size);
          _graph->emplace_back(
            costs[i * j],
            index);
        }
      }
    }
  } else {
    for (unsigned int i = 0; i != x; i++) {
      for (unsigned int j = 0; j != y; j++) {
        for (unsigned int k = 0; k != _dim3_size; k++) {
          // Optimization: operator[] is used over at() for performance (no bound checking)
          index = NodeSE2::getIndex(i, j, k, _x_size, _dim3_size);
          _graph->operator[](index).reset(
            costs[i * j],
            index);
        }
      }
    }
  }
}

template <>
void AStarAlgorithm<Node2D>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & theta)
{
  if (theta != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting angle.");
  }
  unsigned int index = Node2D::getIndex(mx, my, getSizeX());
  _start = & _graph->operator[](index);
}

template <>
void AStarAlgorithm<NodeSE2>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & theta)
{
  unsigned int index = NodeSE2::getIndex(mx, my, theta, getSizeX(), getSizeDim3());
  _start = & _graph->operator[](index);
  _start->setPose(Pose(mx, my, theta));
}

template <>
void AStarAlgorithm<Node2D>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & theta)
{
  if (theta != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal angle.");
  }

  unsigned int index = Node2D::getIndex(mx, my, getSizeX());
  _goal = & _graph->operator[](index);
  _goal_coordinates = Node2D::Coordinates(mx, my);
}

template <>
void AStarAlgorithm<NodeSE2>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & theta)
{
  unsigned int index = NodeSE2::getIndex(mx, my, theta, getSizeX(), getSizeDim3());
  _goal = & _graph->operator[](index);
  _goal_coordinates = NodeSE2::Coordinates(mx, my, theta);
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // Check if initialization was called
  if (!_graph) {
    throw std::runtime_error("Failed to compute path, initialization not called.");
  }

  // Check if graph was filled in
  if (_graph->empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 && !_goal->isNodeValid(_traverse_unknown)) {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_traverse_unknown)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(IndexPath & path, int & iterations, const float & tolerance)
{  
  if (!areInputsValid()) {
    return false;
  }

  _tolerance = _neutral_cost * tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  // 0) Add starting point to the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  NodePtr current_node;
  float g_cost;
  NodeVector neighbors;
  int approach_iterations = 0;
  typename NodeVector::iterator neighbor_iterator;

  // Given an index, return a node ptr reference if its collision-free and valid
  std::function<bool(const unsigned int&, NodeT*&)> node_validity_checker =
  [&, this](const unsigned int & index, NodePtr & neighbor) -> bool
  {
      neighbor = & _graph->operator[](index);
      if (neighbor->isNodeValid(_traverse_unknown))
      {
        return true;
      }

      return false;
  };

  while (iterations < getMaxIterations() && !_queue->empty()) {

    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return backtracePath(current_node, path);
    }
    // else if (_best_heuristic_node.first < getToleranceHeuristic()) {
    //   // Optimization: Let us find when in tolerance and refine within reason
    //   approach_iterations++;
    //   if (approach_iterations > getOnApproachMaxIterations() ||
    //     iterations + 1 == getMaxIterations())
    //   {
    //     NodePtr node = & _graph->operator[](_best_heuristic_node.second);
    //     return backtracePath(node, path);
    //   }
    // }
    // TODO STEVE removed, causing issues in NodeSE2

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    NodeT::getNeighbors(current_node, node_validity_checker, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      NodePtr & neighbor = * neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      g_cost = current_node->getAccumulatedCost() +
        getTraversalCost(current_node, neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) If not in queue or visited, add it
        if (!neighbor->wasVisited()) {
          neighbor->queued();
          addNode(g_cost + getHeuristicCost(neighbor), neighbor);
        }
      }
    }
  }

  return false;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template <>
bool AStarAlgorithm<Node2D>::backtracePath(NodePtr & node, IndexPath & path)
{
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  while (current_node->parent) {
    Node2D::Coordinates coords = Node2D::getCoords(
      current_node->getIndex(), getSizeX(), getSizeDim3());
    path.emplace_back(
      static_cast<float>(coords.x), static_cast<float>(coords.y));
    current_node = current_node->parent;
  }

  return path.size() > 1;
}

template <>
bool AStarAlgorithm<NodeSE2>::backtracePath(NodePtr & node, IndexPath & path)
{
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  // TODO(stevemacenski): return orientation (important?)
  while (current_node->parent) {
    const Pose node_continuous_pose = current_node->pose;
    path.push_back({node_continuous_pose._x, node_continuous_pose._y});
    current_node = current_node->parent;
  }

  return path.size() > 1;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNode()
{
  NodePtr node = _queue->top().second;
  _queue->pop();
  return node;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float cost, NodePtr & node)
{
  _queue->emplace(cost, node);
}

// TODO does this need to change for SE3 Node? Perhaps be footprint cost now?
// Might need a new NodeT::getTravelCost() so each can have their own to get cost
// TODO g cost include change direction + forward / back penalties.
// https://github.com/karlkurzer/path_planner/blob/master/src/node3d.cpp#L73-L102
template<typename NodeT>
float AStarAlgorithm<NodeT>::getTraversalCost(
  NodePtr & current_node,
  NodePtr & new_node)
{
  float & node_cost = new_node->getCost();

  // rescale cost quadratically, makes search more convex
  // Higher the scale, the less cost for lengthwise expansion
  return _neutral_cost + _travel_cost_scale * node_cost * node_cost;
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  Coordinates node_coords = NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(
    node_coords, _goal_coordinates) * _neutral_cost;
  
  // If we're far from goal, we want to ensure we can speed it along
  if (heuristic > getToleranceHeuristic()) {
    heuristic *= _neutral_cost;
  }

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  while (!_queue->empty()) {
    _queue->pop();
  }
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

// Instantiate AStartAlgorithm for the supported template type parameters
// This is needed to prevent "undefined symbol" errors at runtime.
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeSE2>;

}  // namespace smac_planner
