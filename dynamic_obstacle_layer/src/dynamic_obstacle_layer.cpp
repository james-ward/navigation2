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

// Author: Steve Macenski

#include "dynamic_obstacle_layer/dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

PLUGINLIB_EXPORT_CLASS(dynamic_obstacle_layer::DynamicObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace dynamic_obstacle_layer
{

// costmap 2d boundry updates support clusters
  // when something is far off from anything else (like this seeing 20 m out)

// openMP / GPU optimized voxel / inflation / static / range / obstacle / STVL layers
  // AMCL
  // DWB
  // TEB
  // voxel grid
  // height map (grid_map, openVDB wrap, application itself, data processors, ray casting)
  // TF transformations for pointclouds / laser scans / etc
  // laser geometry projection
  // anything with poitncloud, laser scans, images, costmaps, anything iterating over large data structures

// convolution / tooplogical costmap layers for removing noise
// costmap / height map is now probablistic like a light weight octomap, no more word of god sensors measurements


DynamicObstacleLayer::~DynamicObstacleLayer()
{
  // reset ptrs, kill threads, stop processing

  // costmaplayer or just a layer?

  // take in multiple track observations or just the most recent one? If multiple, merging?
}

void DynamicObstacleLayer::onInitialize()
{
  // get params

  // respect maximum range / size parameter of costmap and layer.

  // create message filter subscriber, making sure it can be transformed into the global frame

  // different models for projection (elipsoid/guassian process, straight smear, uncertainty projection)

  // visualize: cylinders of size with velocity arrow
}

void DynamicObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  // if (rolling_window_) {
  //   // TODO what to do if rolling / local costmap
  //   updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  // }
  if (!enabled_) {
    return;
  }
  (void)min_x;
  (void)min_y;
  (void)max_x;
  (void)max_y;
  (void)robot_yaw;
  (void)robot_y;
  (void)robot_x;
  // get list of current tracks (pose, time, size, velocity)

  // some time stuff (interpolation, etc) for tracks older to where tehy would be now, up until a timeout for too old.

  // touch the points going to mark

  // get bounding box points of where we might update
}

void DynamicObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  (void)min_i;
  (void)min_j;
  (void)max_i;
  (void)max_j;
  (void)master_grid;

  // mark obstacles

  // project and mark where going to be with some function

  // openMP process each?
}

void DynamicObstacleLayer::activate()
{
  // activate subscriber, enable state
}

void DynamicObstacleLayer::deactivate()
{
  // deactivate subscriber, disable state
}

void DynamicObstacleLayer::reset()
{
  deactivate();
  //tracks.clear();
  activate();
}

}  // namespace nav2_costmap_2d
