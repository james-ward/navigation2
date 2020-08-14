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

#ifndef SMAC_PLANNER__NODE_2D_HPP_
#define SMAC_PLANNER__NODE_2D_HPP_

#include <vector>
#include <iostream>
#include <queue>
#include <limits>

#include "smac_planner/constants.hpp"

namespace smac_planner
{

/**
 * @class smac_planner::Node2D
 * @brief Node2D implementation for graph
 */
class Node2D
{
public:

  /**
   * @class smac_planner::Node2D::Coordinates
   * @brief Node2D implementation of coordinate structure
   */
  struct Coordinates
  {
    Coordinates() {};
    Coordinates(const float & x_in, const float & y_in)
    : x(x_in), y(y_in)
    {};
    
    float x, y; 
  };

  /**
   * @brief A constructor for smac_planner::Node2D
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit Node2D(unsigned char & cost_in, const unsigned int index)
  : parent(nullptr),
    _cell_cost(static_cast<float>(cost_in)),
    _accumulated_cost(std::numeric_limits<float>::max()),
    _index(index),
    _was_visited(false),
    _is_queued(false)
  {
  }

  /**
   * @brief A destructor for smac_planner::Node2D
   */
  ~Node2D()
  {
    parent = nullptr;
  }

  /**
   * @brief operator== for comparisons
   * @param Node2D right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const Node2D & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief Reset method for new search
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  inline void reset(const unsigned char & cost, const unsigned int index)
  {
    parent = nullptr;
    _cell_cost = static_cast<float>(cost);
    _accumulated_cost = std::numeric_limits<float>::max();
    _index = index;
    _was_visited = false;
    _is_queued = false;
  }

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  inline void setAccumulatedCost(const float cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline float & getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  inline bool & wasVisited()
  {
    return _was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  inline void visited()
  {
    _was_visited = true;
    _is_queued = false;
  }

  /**
   * @brief Gets if cell is currently queued in search
   * @param If cell was queued
   */
  inline bool & isQueued()
  {
    return _is_queued;
  }

  /**
   * @brief Sets if cell is currently queued in search
   */
  inline void queued()
  {
    _is_queued = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
  inline unsigned int & getIndex()
  {
    return _index;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @return whether this node is valid and collision free
   */
  inline bool isNodeValid(const bool & traverse_unknown) {
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

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & width)
  {
    return x + y * width;
  }

  static inline Coordinates getCoords(
    const unsigned int & index, const unsigned int & width, const unsigned int & angles)
  {
    if (angles != 1) {
      throw std::runtime_error("Node type Node2D does not have a valid angle quantization.");
    }

    return Coordinates(index % width, index / width);
  }

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  static inline float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates,
    const float & neutral_cost)
  {
    return hypotf(
      goal_coordinates.x - node_coords.x,
      goal_coordinates.y - node_coords.y) * neutral_cost;
  }

  Node2D * parent;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NODE_2D_HPP_
