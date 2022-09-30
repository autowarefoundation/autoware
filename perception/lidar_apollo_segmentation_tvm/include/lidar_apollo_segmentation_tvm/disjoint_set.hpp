// Copyright 2017-2022 Arm Ltd., The Apollo Authors
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__DISJOINT_SET_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__DISJOINT_SET_HPP_

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
/// \brief Add a new element in a new set.
/// \param[inout] x The element to be added.
template <class T>
void DisjointSetMakeSet(T * x)
{
  x->parent = x;
  x->node_rank = 0;
}

/// \brief Recursively follow the chain of parent pointers from the input until reaching a root.
/// \param[inout] x The element which root is looked for.
/// \return The root of the set containing x.
template <class T>
T * DisjointSetFindRecursive(T * x)
{
  if (x->parent != x) {
    x->parent = DisjointSetFindRecursive(x->parent);
  }
  return x->parent;
}

/// \brief Find the root of the set x belongs to.
/// \param[inout] x The set element.
/// \return The root of the set containing x.
template <class T>
T * DisjointSetFind(T * x)
{
  T * y = x->parent;
  if (y == x || y->parent == y) {
    return y;
  }
  T * root = DisjointSetFindRecursive(y->parent);
  x->parent = root;
  y->parent = root;
  return root;
}

/// \brief Replace the set containing x and the set containing y with their union.
/// \param[inout] x An element of a first set.
/// \param[inout] y An element of a second set.
template <class T>
void DisjointSetUnion(T * x, T * y)
{
  x = DisjointSetFind(x);
  y = DisjointSetFind(y);
  if (x == y) {
    return;
  }
  if (x->node_rank < y->node_rank) {
    x->parent = y;
  } else if (y->node_rank < x->node_rank) {
    y->parent = x;
  } else {
    y->parent = x;
    x->node_rank++;
  }
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__DISJOINT_SET_HPP_
