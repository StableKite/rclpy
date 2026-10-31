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

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "clock.hpp"

using pybind11::literals::operator""_a;

namespace rclpy
{
namespace
{
bool thread_attached()
{
#if PY_VERSION_HEX >= 0x030D0000
  return PyThreadState_GetUnchecked() != nullptr;
#else
  return PyGILState_Check() != 0;
#endif
}

bool python_finalizing()
{
#if PY_VERSION_HEX >= 0x030D0000
  return Py_IsFinalizing() != 0;
#else
  return _Py_IsFinalizing() != 0;
#endif
}

void on_time_jump(const rcl_time_jump_t *, bool, void *) noexcept;

struct ClockCallback
{
  ClockCallback(const std::shared_ptr<ClockState> & state, py::object owner)
  : state(state), owner(owner) {}

  std::weak_ptr<ClockState> state;
  py::weakref owner;
  bool enabled = true;
};
}  // namespace

class ClockState
{
public:
  explicit ClockState(int clock_type)
  : lock(py::module_::import("threading").attr("RLock")()),
    acquire(lock.attr("acquire")), release(lock.attr("release"))
  {
    auto allocator = rcl_get_default_allocator();
    if (RCL_RET_OK != rcl_clock_init(rcl_clock_type_t(clock_type), &clock, &allocator)) {
      throw RCLError("failed to initialize clock");
    }
    initialized = true;
  }

  ~ClockState()
  {
    // A worker can release the last native lease. Attach before DECREF, but
    // never attach a foreign thread to an interpreter that is shutting down.
    const bool attached = thread_attached();
    std::optional<py::gil_scoped_acquire> attachment;
    if (!attached && Py_IsInitialized() && !python_finalizing()) {
      attachment.emplace();
    }
    const bool can_use_python = attached || attachment.has_value();
    if (!can_use_python) {
      for (auto & callback : callbacks) {
        (void)callback->owner.release();
      }
      (void)lock.release();
      (void)acquire.release();
      (void)release.release();
    }
    callbacks.clear();
    lock = py::object();
    acquire = py::object();
    release = py::object();
    if (initialized && RCL_RET_OK != rcl_clock_fini(&clock)) {
      // Cleanup must not replace an exception already being propagated.
      if (can_use_python) {
        PyObject * type = nullptr;
        PyObject * value = nullptr;
        PyObject * traceback = nullptr;
        PyErr_Fetch(&type, &value, &traceback);
        if (PyErr_WarnFormat(PyExc_RuntimeWarning, 1, "Failed to fini clock: %s",
            rcl_get_error_string().str) < 0)
        {
          PyErr_WriteUnraisable(Py_None);
        }
        PyErr_Restore(type, value, traceback);
      } else {
        std::fprintf(stderr, "rclpy clock cleanup failed: %s\n", rcl_get_error_string().str);
      }
      rcl_reset_error();
    }
  }

  // Called only with lock held, outside the SDK's callback-array traversal.
  void prune_callbacks()
  {
    for (auto it = callbacks.begin(); it != callbacks.end(); ) {
      auto & callback = *it;
      if (callback->enabled && !callback->owner().is_none()) {
        ++it;
        continue;
      }
      auto ret = rcl_clock_remove_jump_callback(&clock, on_time_jump, callback.get());
      // RCL can report allocation failure AFTER removing an entry, when its
      // attempt to shrink the array fails. Do not leave freed userdata behind.
      bool still_registered = false;
      for (size_t i = 0; i < clock.num_jump_callbacks; ++i) {
        const auto & entry = clock.jump_callbacks[i];
        if (entry.callback == on_time_jump && entry.user_data == callback.get()) {
          still_registered = true;
          break;
        }
      }
      if (!still_registered) {
        it = callbacks.erase(it);
      } else {
        ++it;
      }
      if (RCL_RET_OK != ret) {
        throw RCLError("failed to remove time jump callback");
      }
    }
  }

  rcl_clock_t clock{};
  bool initialized = false;
  py::object lock;
  py::object acquire;
  py::object release;
  std::vector<std::shared_ptr<ClockCallback>> callbacks;
  std::vector<std::weak_ptr<void>> timers;
  std::vector<std::shared_ptr<void>> active_timers;
  size_t update_depth = 0;
};

ClockMutation::ClockMutation(std::shared_ptr<ClockState> state, bool cleanup)
: state_(std::move(state))
{
  if (!state_) {
    throw InvalidHandle("cannot use a destroyed clock");
  }
  release_ = state_->release;
  if (cleanup) {
    // Fini cannot be abandoned halfway through a resource lifetime. Preserve
    // an active exception and retry an interrupted wait before removing the
    // timer's SDK callback. Bound lock methods were allocated at clock init.
    py::error_scope preserve;
    for (;;) {
      try {
        state_->acquire();
        break;
      } catch (py::error_already_set & error) {
        error.discard_as_unraisable("acquiring the clock guard for native cleanup");
      }
    }
  } else {
    state_->acquire();
  }
  locked_ = true;
}

ClockMutation::~ClockMutation() noexcept
{
  if (locked_) {
    PyObject * type = nullptr;
    PyObject * value = nullptr;
    PyObject * traceback = nullptr;
    PyErr_Fetch(&type, &value, &traceback);
    try {
      release_();
    } catch (py::error_already_set & error) {
      error.discard_as_unraisable("releasing the clock mutation lock");
    }
    PyErr_Restore(type, value, traceback);
  }
}

rcl_clock_t * ClockMutation::rcl_ptr() const
{
  return &state_->clock;
}

void ClockMutation::track_timer(const std::shared_ptr<void> & owner)
{
  auto & timers = state_->timers;
  timers.erase(std::remove_if(timers.begin(), timers.end(),
      [](const auto & weak) {return weak.expired();}), timers.end());
  timers.push_back(owner);
  if (state_->update_depth != 0) {
    // A timer created by a jump callback must survive the rest of the SDK
    // traversal too: timer fini removes an entry from that very same array.
    state_->active_timers.push_back(owner);
  }
}

namespace
{
class ClockUpdate
{
public:
  explicit ClockUpdate(ClockState & state) : state_(state)
  {
    if (state_.update_depth == 0) {
      state_.prune_callbacks();
      std::vector<std::shared_ptr<void>> owners;
      for (const auto & weak : state_.timers) {
        if (auto owner = weak.lock()) {
          owners.push_back(std::move(owner));
        }
      }
      state_.active_timers.swap(owners);
    }
    ++state_.update_depth;
  }

  ~ClockUpdate()
  {
    if (!finished_) {
      try {
        finish();
      } catch (py::error_already_set & error) {
        error.discard_as_unraisable("cleaning up clock callbacks");
      } catch (const std::exception & error) {
        std::fprintf(stderr, "rclpy clock callback cleanup failed: %s\n", error.what());
      }
    }
  }

  void finish()
  {
    finished_ = true;
    if (--state_.update_depth == 0) {
      std::vector<std::shared_ptr<void>> owners;
      owners.swap(state_.active_timers);
      state_.prune_callbacks();
      // Release timers only after callback iteration and pruning have ended.
    }
  }

private:
  ClockState & state_;
  bool finished_ = false;
};

template<typename Function>
void update_clock(ClockState & state, Function && function)
{
  ClockUpdate update(state);
  const auto ret = function();
  std::optional<py::error_already_set> callback_error;
  if (PyErr_Occurred()) {
    callback_error.emplace();
  }
  std::optional<RCLError> native_error;
  if (RCL_RET_OK != ret) {
    native_error.emplace("failed to set ROS time override");
  }
  std::exception_ptr cleanup_error;
  try {
    update.finish();
  } catch (...) {
    cleanup_error = std::current_exception();
  }
  if (callback_error) {
    throw std::move(*callback_error);
  }
  if (native_error) {
    throw *native_error;
  }
  if (cleanup_error) {
    std::rethrow_exception(cleanup_error);
  }
}

void on_time_jump(const rcl_time_jump_t * time_jump, bool before_jump, void * user_data) noexcept
{
  const bool already_attached = thread_attached();
  py::gil_scoped_acquire attached;
  if (PyErr_Occurred()) {
    // Preserve the first failure before making any further Python calls.
    return;
  }
  try {
    auto & callback = *static_cast<ClockCallback *>(user_data);
    auto state = callback.state.lock();
    if (!state) {
      return;
    }
    ClockMutation guard(state);
    if (PyErr_Occurred() || !callback.enabled) {
      return;
    }
    auto owner = callback.owner();
    if (owner.is_none()) {
      return;
    }
    auto function = owner.attr(before_jump ? "_pre_callback" : "_post_callback");
    if (function.is_none()) {
      return;
    }
    if (before_jump) {
      function();
    } else {
      function(py::dict("clock_change"_a = py::cast(time_jump->clock_change),
          "delta"_a = time_jump->delta.nanoseconds));
    }
  } catch (py::error_already_set & error) {
    error.restore();
  } catch (const std::exception & error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "unknown exception in clock jump callback");
  }
  if (!already_attached && PyErr_Occurred()) {
    PyErr_WriteUnraisable(Py_None);
  }
}
}  // namespace

Clock::Clock(int clock_type)
: state_(std::make_shared<ClockState>(clock_type)) {}

Clock::Clock(const Clock & other)
: Destroyable(other), std::enable_shared_from_this<Clock>(),
  state_(std::atomic_load(&other.state_)) {}

void Clock::destroy()
{
  std::atomic_store(&state_, std::shared_ptr<ClockState>());
}

std::shared_ptr<rcl_clock_t> Clock::rcl_handle() const
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed clock");
  }
  return std::shared_ptr<rcl_clock_t>(state, &state->clock);
}

rcl_clock_t * Clock::rcl_ptr() const
{
  return rcl_handle().get();
}

ClockMutation Clock::mutation() const
{
  return ClockMutation(std::atomic_load(&state_));
}

rcl_time_point_t Clock::get_now()
{
  auto clock = rcl_handle();
  rcl_time_point_t time_point;
  time_point.clock_type = clock->type;
  if (RCL_RET_OK != rcl_clock_get_now(clock.get(), &time_point.nanoseconds)) {
    throw RCLError("failed to get current clock value");
  }
  return time_point;
}

bool Clock::get_ros_time_override_is_enabled()
{
  auto clock = rcl_handle();
  bool enabled;
  if (RCL_RET_OK != rcl_is_enabled_ros_time_override(clock.get(), &enabled)) {
    throw RCLError("failed to get ROS time override status");
  }
  return enabled;
}

void Clock::set_ros_time_override_is_enabled(bool enabled)
{
  auto guard = mutation();
  update_clock(*guard.state_, [&]() {
      return enabled ? rcl_enable_ros_time_override(guard.rcl_ptr()) :
             rcl_disable_ros_time_override(guard.rcl_ptr());
    });
}

void Clock::set_ros_time_override(rcl_time_point_t time_point)
{
  auto guard = mutation();
  update_clock(*guard.state_, [&]() {
      return rcl_set_ros_time_override(guard.rcl_ptr(), time_point.nanoseconds);
    });
}

void Clock::add_clock_callback(
  py::object pyjump_handle,
  bool on_clock_change,
  int64_t min_forward,
  int64_t min_backward)
{
  auto guard = mutation();
  auto state = guard.state_;
  if (state->update_depth == 0) {
    state->prune_callbacks();
  }
  for (const auto & callback : state->callbacks) {
    if (callback->enabled && callback->owner().is(pyjump_handle)) {
      RCL_SET_ERROR_MSG("callback/user_data are already added to this clock");
      throw RCLError("failed to add time jump callback");
    }
  }
  auto callback = std::make_shared<ClockCallback>(state, pyjump_handle);
  state->callbacks.push_back(callback);
  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = on_clock_change;
  threshold.min_forward.nanoseconds = min_forward;
  threshold.min_backward.nanoseconds = min_backward;

  rcl_ret_t ret = rcl_clock_add_jump_callback(
    guard.rcl_ptr(), threshold, on_time_jump, callback.get());
  if (ret != RCL_RET_OK) {
    state->callbacks.pop_back();
    throw RCLError("failed to add time jump callback");
  }
}

void Clock::remove_clock_callback(py::object pyjump_handle)
{
  auto guard = mutation();
  auto state = guard.state_;
  for (auto & callback : state->callbacks) {
    if (callback->enabled && callback->owner().is(pyjump_handle)) {
      callback->enabled = false;
      if (state->update_depth == 0) {
        // Keep this record alive if pruning erases its registry entry.
        auto removing = callback;
        try {
          state->prune_callbacks();
        } catch (...) {
          if (std::find(state->callbacks.begin(), state->callbacks.end(), removing) !=
            state->callbacks.end())
          {
            removing->enabled = true;
          }
          throw;
        }
      }
      return;
    }
  }
  RCL_SET_ERROR_MSG("jump callback was not found");
  throw RCLError("failed to remove time jump callback");
}

void Clock::unregister_clock_callback(py::object pyjump_handle)
{
  auto guard = mutation();
  if (!pyjump_handle.attr("_clock").is_none()) {
    try {
      remove_clock_callback(pyjump_handle);
    } catch (...) {
      // A failed SDK shrink can still have completed removal. Reflect that
      // committed change in the Python handle while propagating the error.
      bool registered = false;
      for (const auto & callback : guard.state_->callbacks) {
        if (callback->owner().is(pyjump_handle)) {
          registered = true;
          break;
        }
      }
      if (!registered) {
        pyjump_handle.attr("_clock") = py::none();
      }
      throw;
    }
    pyjump_handle.attr("_clock") = py::none();
  }
}

void define_clock(py::object module)
{
  py::class_<Clock, Destroyable, std::shared_ptr<Clock>>(module, "Clock")
  .def(py::init<int>())
  .def_property_readonly(
    "pointer", [](const Clock & clock) {
      return reinterpret_cast<size_t>(clock.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_now", &Clock::get_now,
    "Current value of the clock")
  .def(
    "get_ros_time_override_is_enabled",
    &Clock::get_ros_time_override_is_enabled,
    "Returns if a clock using ROS time has the ROS time override enabled.")
  .def(
    "set_ros_time_override_is_enabled",
    &Clock::set_ros_time_override_is_enabled,
    "Set if a clock using ROS time has the ROS time override enabled.")
  .def(
    "set_ros_time_override",
    &Clock::set_ros_time_override,
    "Set the ROS time override for a clock using ROS time.")
  .def(
    "add_clock_callback",
    &Clock::add_clock_callback,
    "Add a time jump callback to a clock.")
  .def(
    "unregister_clock_callback", &Clock::unregister_clock_callback,
    "Remove a Python JumpHandle once, including concurrent or reentrant unregister.")
  .def(
    "remove_clock_callback",
    &Clock::remove_clock_callback,
    "Remove a time jump callback from a clock.");
}
}  // namespace rclpy
