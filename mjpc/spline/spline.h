// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MJPC_MJPC_SPLINE_SPLINE_H_
#define MJPC_MJPC_SPLINE_SPLINE_H_

#include <cstddef>
#include <deque>
#include <vector>

#include <absl/types/span.h>

namespace mjpc::spline {

// Represents a spline where values are interpolated based on time.
// Allows updating the spline by adding new future points, or removing old
// nodes.
// This class is not thread safe and requires locking to use.
class TimeSpline {
 public:
  explicit TimeSpline(int dim = 0, int initial_capacity = 1);

  // Copyable, Movable.
  TimeSpline(const TimeSpline& other) = default;
  TimeSpline& operator=(const TimeSpline& other) = default;
  TimeSpline(TimeSpline&& other) = default;
  TimeSpline& operator=(TimeSpline&& other) = default;

  // A view into one spline node in the spline.
  // Template parameter is needed to support both `double` and `const double`
  // views of the data.
  template <typename T>
  class NodeT {
   public:
    NodeT() : time_(0) {};
    NodeT(double time, T* values, int dim)
        : time_(time), values_(values, dim) {}

    // Copyable, Movable.
    NodeT(const NodeT& other) = default;
    NodeT& operator=(const NodeT& other) = default;
    NodeT(NodeT&& other) = default;
    NodeT& operator=(NodeT&& other) = default;

    double time() const { return time_; }

    // Returns a span pointing to the spline values of the node.
    // This function returns a non-const span, to allow spline values to be
    // modified, while the time member and underlying values pointer remain
    // constant.
    absl::Span<T> values() const { return values_; }

   private:
    double time_;
    absl::Span<T> values_;
  };

  using Node = NodeT<double>;
  using ConstNode = NodeT<const double>;

  // Returns the number of nodes in the spline.
  std::size_t Size() const;

  // Returns the node at the given index, sorted by time. Any calls that mutate
  // the spline will invalidate the Node object.
  Node NodeAt(int index);
  ConstNode NodeAt(int index) const;

  // Returns the dimensionality of interpolation values.
  int Dim() const;

  // Reserves memory for at least num_nodes. If the spline already contains
  // more nodes, does nothing.
  void Reserve(int num_nodes);

  // Interpolates values based on time, writes results to `values`.
  // NOTE: The current implementation does a "zero-order interpolation". Linear
  // and cubic interpolations will be added in a follow-up.
  void Sample(double time, absl::Span<double> values) const;
  // Interpolates values based on time, returns a vector of length Dim.
  std::vector<double> Sample(double time) const;

  // Removes any old nodes that have no effect on the values at time `time`.
  // Returns the number of nodes removed.
  int DiscardBefore(double time);

  // Removes all existing nodes.
  void Clear();

  // Adds a new set of values at the given time.
  // This class only supports adding nodes with a time later or earlier than
  // all other nodes.
  Node AddNode(double time);
  Node AddNode(double time, absl::Span<const double> values);

 private:
  int dim_;

  // The time values for each node. This is kept sorted.
  std::deque<double> times_;

  // The raw node values. Stored in a ring buffer, which is resized whenever
  // too many nodes are added.
  std::vector<double> values_;

  // The index in values_ for the data of the earliest node.
  int values_begin_ = 0;

  // One past the index in values_ for the end of the data of the last node.
  // If values_end_ == values_begin_, either there's no data (nodes_ is empty),
  // or the values_ buffer is full.
  int values_end_ = 0;
};

}  // namespace mjpc::spline

#endif  // MJPC_MJPC_SPLINE_SPLINE_H_
