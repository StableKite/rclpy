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
#include <rcl/guard_condition.h>
#include <rcl/types.h>

#include <atomic>
#include <cstdio>
#include <memory>

#include "context.hpp"
#include "exceptions.hpp"
#include "guard_condition.hpp"

namespace rclpy
{
class GuardConditionState
{
public:
  explicit GuardConditionState(Context & owner)
  : context(owner.rcl_handle())
  {
    auto options = rcl_guard_condition_get_default_options();
    if (RCL_RET_OK != rcl_guard_condition_init(&guard, context.get(), options)) {
      throw RCLError("failed to create guard_condition");
    }
    initialized = true;
  }

  ~GuardConditionState()
  {
    if (initialized && RCL_RET_OK != rcl_guard_condition_fini(&guard)) {
#if PY_VERSION_HEX >= 0x030D0000
      const bool attached = Py_IsInitialized() && PyThreadState_GetUnchecked() != nullptr;
#else
      const bool attached = Py_IsInitialized() && PyGILState_Check();
#endif
      if (attached) {
        py::error_scope preserve;
        if (PyErr_WarnFormat(PyExc_RuntimeWarning, 1, "Failed to fini guard condition: %s",
            rcl_get_error_string().str) < 0)
        {
          PyErr_WriteUnraisable(Py_None);
        }
      } else {
        std::fprintf(stderr, "rclpy guard condition cleanup failed: %s\n",
          rcl_get_error_string().str);
      }
      rcl_reset_error();
    }
  }

  std::shared_ptr<rcl_context_t> context;
  rcl_guard_condition_t guard = rcl_get_zero_initialized_guard_condition();
  bool initialized = false;
};

GuardCondition::GuardCondition(Context & context)
: state_(std::make_shared<GuardConditionState>(context)) {}

GuardCondition::GuardCondition(const GuardCondition & other)
: Destroyable(other), std::enable_shared_from_this<GuardCondition>(),
  state_(std::atomic_load(&other.state_)) {}

std::shared_ptr<rcl_guard_condition_t> GuardCondition::rcl_handle() const
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed guard condition");
  }
  return std::shared_ptr<rcl_guard_condition_t>(state, &state->guard);
}

rcl_guard_condition_t * GuardCondition::rcl_ptr() const
{
  return rcl_handle().get();
}

void
GuardCondition::destroy()
{
  std::atomic_store(&state_, std::shared_ptr<GuardConditionState>());
}

void
GuardCondition::trigger_guard_condition()
{
  auto guard = rcl_handle();
  rcl_ret_t ret = rcl_trigger_guard_condition(guard.get());

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to trigger guard condition");
  }
}

void define_guard_condition(py::object module)
{
  py::class_<GuardCondition, Destroyable, std::shared_ptr<GuardCondition>>(module, "GuardCondition")
  .def(py::init<Context &>())
  .def_property_readonly(
    "pointer", [](const GuardCondition & guard_condition) {
      return reinterpret_cast<size_t>(guard_condition.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "trigger_guard_condition", &GuardCondition::trigger_guard_condition,
    "Trigger a general purpose guard condition");
}
}  // namespace rclpy
