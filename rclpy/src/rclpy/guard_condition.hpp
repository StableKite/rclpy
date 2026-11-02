// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__GUARD_CONDITION_HPP_
#define RCLPY__GUARD_CONDITION_HPP_

#include <pybind11/pybind11.h>

#include <rcl/guard_condition.h>

#include <memory>

#include "context.hpp"
#include "destroyable.hpp"

namespace py = pybind11;

namespace rclpy
{
class GuardConditionState;
/// Create a general purpose guard condition
class GuardCondition : public Destroyable, public std::enable_shared_from_this<GuardCondition>
{
public:
  /**
   * Raises RuntimeError if initializing the guard condition fails
   */
  explicit GuardCondition(Context & context);
  GuardCondition(const GuardCondition & other);

  /// Trigger a general purpose guard condition
  /**
   * Raises ValueError if pygc is not a guard condition capsule
   * Raises RCLError if the guard condition could not be triggered
   */
  void
  trigger_guard_condition();

  /// Get rcl_guard_condition_t pointer
  rcl_guard_condition_t * rcl_ptr() const;

  /// Keep the guard and its native context alive during a deferred trigger.
  std::shared_ptr<rcl_guard_condition_t> rcl_handle() const;

  /// Force an early destruction of this object
  void destroy() override;

private:
  std::shared_ptr<GuardConditionState> state_;
};

/// Define a pybind11 wrapper for an rclpy::GuardCondition
void define_guard_condition(py::object module);
}  // namespace rclpy

#endif  // RCLPY__GUARD_CONDITION_HPP_
