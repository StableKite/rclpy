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

#ifndef RCLPY__CLOCK_HPP_
#define RCLPY__CLOCK_HPP_

#include <pybind11/pybind11.h>

#include <rcl/time.h>

#include <memory>

#include "destroyable.hpp"
#include "exceptions.hpp"

namespace py = pybind11;

namespace rclpy
{
class ClockState;

/// Own a native clock while serializing mutations that invoke Python callbacks.
class ClockMutation
{
public:
  explicit ClockMutation(std::shared_ptr<ClockState> state, bool cleanup = false);
  ClockMutation(const ClockMutation &) = delete;
  ClockMutation & operator=(const ClockMutation &) = delete;
  ~ClockMutation() noexcept;

  rcl_clock_t * rcl_ptr() const;
  void track_timer(const std::shared_ptr<void> & owner);

private:
  friend class Clock;
  // Attach before touching Python references. RLock waits cooperate with the
  // Python runtime; no C++ mutex is held while invoking user jump callbacks.
  py::gil_scoped_acquire attached_;
  std::shared_ptr<ClockState> state_;
  py::object release_;
  bool locked_ = false;
};

class Clock : public Destroyable, public std::enable_shared_from_this<Clock>
{
public:
  /// Create a clock
  /**
   * Raises RCLError on initialization failure
   *
   * \param[in] clock_type enum of type ClockType
   * This constructor creates a Clock object
   */
  explicit Clock(int clock_type);
  Clock(const Clock & other);

  /// Returns the current value of the clock
  /**
   * Raises RCLError if the clock's value cannot be retrieved
   *
   * \return capsule to a time point with the current time
   */
  rcl_time_point_t
  get_now();

  /// Returns if a clock using ROS time has the ROS time override enabled.
  /**
   * Raises RCLError if the clock's ROS time override state cannot be retrieved
   *
   * \return true if ROS time is enabled
   */
  bool
  get_ros_time_override_is_enabled();

  /// Set if a clock using ROS time has the ROS time override enabled.
  /**
   * Raises RCLError if the clock's ROS time override cannot be set
   *
   * \param[in] enabled Override state to set
   */
  void
  set_ros_time_override_is_enabled(bool enabled);

  /// Set the ROS time override for a clock using ROS time.
  /**
   * Raises RRCLError if the clock's ROS time override cannot be set
   *
   * \param[in] time_point a time point instance
   */
  void
  set_ros_time_override(rcl_time_point_t time_point);

  /// Add a time jump callback to a clock.
  /**
   * any argument is invalid
   * Raises RCLError if the callback cannot be added
   *
   * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
   * \param[in] on_clock_change True if callback should be called when ROS time is toggled
   * \param[in] min_forward minimum nanoseconds to trigger forward jump callback
   * \param[in] min_backward minimum negative nanoseconds to trigger backward jump callback
   */
  void
  add_clock_callback(
    py::object pyjump_handle,
    bool on_clock_change,
    int64_t min_forward,
    int64_t min_backward);

  /// Remove a time jump callback from a clock.
  /**
   * Raises RCLError if the callback cannot be added
   *
   * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
   */
  void
  remove_clock_callback(py::object pyjump_handle);

  /// Complete the Python JumpHandle unregister transaction under this clock's guard.
  void unregister_clock_callback(py::object pyjump_handle);

  /// Get rcl_clock_t pointer
  rcl_clock_t * rcl_ptr() const;

  /// Keep the native allocation alive independently of a closing Python owner.
  std::shared_ptr<rcl_clock_t> rcl_handle() const;

  /// Timer initialization/finalization also mutate this clock's callback array.
  ClockMutation mutation() const;

  /// Force an early destruction of this object
  void destroy() override;

private:
  friend class TimerState;
  std::shared_ptr<ClockState> state_;
};

/// Define a pybind11 wrapper for an rclpy::Clock
void define_clock(py::object module);
}  // namespace rclpy

#endif  // RCLPY__CLOCK_HPP_
