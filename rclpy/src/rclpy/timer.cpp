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

#include <rcl/error_handling.h>
#include <rcl/timer.h>
#include <rcl/types.h>

#include <atomic>
#include <cstdio>
#include <memory>
#include <optional>

#include "clock.hpp"
#include "context.hpp"
#include "exceptions.hpp"
#include "timer.hpp"

namespace rclpy
{
class TimerState
{
public:
  TimerState(Clock & clock, Context & context)
  : clock(clock), context(context.rcl_handle()) {}

  ~TimerState()
  {
    if (!initialized) {
      return;
    }
#if PY_VERSION_HEX >= 0x030D0000
    const bool attached = PyThreadState_GetUnchecked() != nullptr;
    const bool finalizing = Py_IsFinalizing() != 0;
#else
    const bool attached = PyGILState_Check() != 0;
    const bool finalizing = _Py_IsFinalizing() != 0;
#endif
    // Normal fini shares the clock's registration guard. An interpreter that
    // has already shut down cannot have live Python callback dispatchers.
    std::optional<ClockMutation> guard;
    if (attached || (Py_IsInitialized() && !finalizing)) {
      guard.emplace(std::atomic_load(&clock.state_), true);
    }
    if (RCL_RET_OK != rcl_timer_fini(&timer)) {
      if (guard) {
        PyObject * type = nullptr;
        PyObject * value = nullptr;
        PyObject * traceback = nullptr;
        PyErr_Fetch(&type, &value, &traceback);
        if (PyErr_WarnFormat(PyExc_RuntimeWarning, 1, "Failed to fini timer: %s",
            rcl_get_error_string().str) < 0)
        {
          PyErr_WriteUnraisable(Py_None);
        }
        PyErr_Restore(type, value, traceback);
      } else {
        std::fprintf(stderr, "rclpy timer cleanup failed: %s\n", rcl_get_error_string().str);
      }
      rcl_reset_error();
    }
  }

  // The resource object, rather than a shared_ptr deleter, owns these leases.
  // Clock's weak timer registry must not keep a strong clock-owning deleter
  // alive after the timer itself has been finalized.
  Clock clock;
  std::shared_ptr<rcl_context_t> context;
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  bool initialized = false;
};

void
Timer::destroy()
{
  std::atomic_store(&state_, std::shared_ptr<TimerState>());
}

Timer::Timer(
  Clock & clock, Context & context, int64_t period_nsec, bool autostart)
{
  auto state = std::make_shared<TimerState>(clock, context);
  auto guard = state->clock.mutation();
  // Allocate lifetime tracking before exposing the timer's internal callback
  // to RCL; allocation failure must not require fini during SDK iteration.
  guard.track_timer(state);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_timer_init2(
    &state->timer, guard.rcl_ptr(), state->context.get(),
    period_nsec, NULL, allocator, autostart);

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to create timer");
  }
  state->initialized = true;
  std::atomic_store(&state_, std::move(state));
}

Timer::Timer(const Timer & other)
: Destroyable(other), std::enable_shared_from_this<Timer>(),
  state_(std::atomic_load(&other.state_)) {}

std::shared_ptr<rcl_timer_t> Timer::rcl_handle() const
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed timer");
  }
  return std::shared_ptr<rcl_timer_t>(state, &state->timer);
}

rcl_timer_t * Timer::rcl_ptr() const
{
  return rcl_handle().get();
}

void Timer::reset_timer()
{
  auto timer = rcl_handle();
  rcl_ret_t ret;
  {
    // The native callback replacement transaction can wait for an in-flight
    // callback. Keep the timer lease, but detach so a concurrent Python GC
    // can stop this caller while the SDK mutex is held by another thread.
    py::gil_scoped_release release;
    ret = rcl_timer_reset(timer.get());
  }
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to reset timer");
  }
}

bool Timer::is_timer_ready()
{
  auto timer = rcl_handle();
  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(timer.get(), &is_ready);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check timer ready");
  }
  return is_ready;
}

void Timer::call_timer()
{
  auto timer = rcl_handle();
  rcl_ret_t ret = rcl_timer_call(timer.get());
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TIMER_CANCELED) {
      throw TimerCancelledError("Timer has been canceled");
    } else {
      throw RCLError("failed to call timer");
    }
  }
}

void Timer::change_timer_period(int64_t period_nsec)
{
  auto timer = rcl_handle();
  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer.get(), period_nsec, &old_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to exchange timer period");
  }
}

std::optional<int64_t> Timer::time_until_next_call()
{
  auto timer = rcl_handle();
  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer.get(), &remaining_time);
  if (ret == RCL_RET_TIMER_CANCELED) {
    return std::nullopt;
  } else if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time until next timer call");
  }

  return remaining_time;
}

int64_t Timer::time_since_last_call()
{
  auto timer = rcl_handle();
  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer.get(), &elapsed_time);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time since last timer call");
  }

  return elapsed_time;
}

int64_t Timer::get_timer_period()
{
  auto timer = rcl_handle();
  int64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(timer.get(), &timer_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get timer period");
  }
  return timer_period;
}

void Timer::cancel_timer()
{
  auto timer = rcl_handle();
  rcl_ret_t ret = rcl_timer_cancel(timer.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to cancel timer");
  }
}

bool Timer::is_timer_canceled()
{
  auto timer = rcl_handle();
  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(timer.get(), &is_canceled);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check if timer is canceled");
  }
  return is_canceled;
}

void
define_timer(py::object module)
{
  py::class_<Timer, Destroyable, std::shared_ptr<Timer>>(module, "Timer")
  .def(py::init<Clock &, Context &, int64_t, bool>())
  .def_property_readonly(
    "pointer", [](const Timer & timer) {
      return reinterpret_cast<size_t>(timer.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def("reset_timer", &Timer::reset_timer, "Reset a timer.")
  .def(
    "is_timer_ready", &Timer::is_timer_ready,
    "Check if a timer as reached timeout.")
  .def(
    "call_timer", &Timer::call_timer,
    "Call a timer and starts counting again.")
  .def(
    "change_timer_period", &Timer::change_timer_period,
    "Set the period of a timer.")
  .def(
    "time_until_next_call", &Timer::time_until_next_call,
    "Get the remaining time before timer is ready.")
  .def(
    "time_since_last_call", &Timer::time_since_last_call,
    "Get the elapsed time since last timer call.")
  .def(
    "get_timer_period", &Timer::get_timer_period,
    "Get the period of a timer.")
  .def(
    "cancel_timer", &Timer::cancel_timer,
    "Cancel a timer.")
  .def(
    "is_timer_canceled", &Timer::is_timer_canceled,
    "Check if a timer is canceled.");
}

}  // namespace rclpy
