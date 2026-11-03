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
#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/init.h>
#include <rcl/init_options.h>
#include <rcl/types.h>
#include <rcutils/logging_macros.h>

#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>
#include <utility>

#include "context.hpp"
#include "exceptions.hpp"
#include "utils.hpp"

namespace rclpy
{
namespace
{
bool python_thread_attached()
{
#if PY_VERSION_HEX >= 0x030D0000
  return PyThreadState_GetUnchecked() != nullptr;
#else
  return PyGILState_Check() != 0;
#endif
}

std::mutex g_context_lifecycle_mutex;

// These callables contain native SDK calls only. The native lock is released
// before Python thread state is reattached, including exceptional returns.
template<typename Function>
auto native_context_call(std::mutex & mutex, Function && function)
{
  std::optional<py::gil_scoped_release> release;
  if (python_thread_attached()) {
    release.emplace();
  }
  std::lock_guard<std::mutex> guard(mutex);
  return function();
}

void warn_context_cleanup(const char * action)
{
  warn_cleanup_error(action, "rcl_context_t", rcl_get_error_string().str);
  rcl_reset_error();
}

}  // namespace

struct ContextState
{
  rcl_context_t context = rcl_get_zero_initialized_context();
  std::mutex mutex;

  ~ContextState()
  {
    // The last strong state reference is gone: no node, alias or shutdown
    // snapshot can still be accessing this native allocation.
    if (context.impl != nullptr) {
      if (rcl_context_is_valid(&context) && RCL_RET_OK != native_context_call(
          g_context_lifecycle_mutex, [this]() {return rcl_shutdown(&context);}))
      {
        warn_context_cleanup("shutdown");
      }
      if (RCL_RET_OK != native_context_call(
          g_context_lifecycle_mutex, [this]() {return rcl_context_fini(&context);}))
      {
        warn_context_cleanup("fini");
      }
    }
  }
};

namespace
{
std::vector<std::weak_ptr<ContextState>> g_contexts;
std::mutex g_contexts_mutex;
}  // namespace

void shutdown_contexts()
{
  std::vector<std::weak_ptr<ContextState>> pending;
  {
    std::lock_guard<std::mutex> guard(g_contexts_mutex);
    pending.swap(g_contexts);
  }
  // No registry lock spans native shutdown, callbacks or state destruction.
  for (const auto & weak : pending) {
    auto state = weak.lock();
    if (!state) {
      continue;
    }
    const auto ret = native_context_call(state->mutex, [&state]() {
        return rcl_context_is_valid(&state->context) ?
               rcl_shutdown(&state->context) : RCL_RET_OK;
      });
    if (ret != RCL_RET_OK) {
      warn_context_cleanup("shutdown contexts");
    }
  }
}

Context::Context(py::list pyargs, size_t domain_id)
{
  auto state = std::make_shared<ContextState>();
  CliArguments arguments(pyargs);
  InitOptions init_options(rcl_get_default_allocator());

  rcl_ret_t ret = rcl_init_options_set_domain_id(&init_options.rcl_options, domain_id);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to set domain id to init options");
  }
  ret = native_context_call(g_context_lifecycle_mutex, [&]() {
      return rcl_init(
        arguments.argc(), arguments.argv(), &init_options.rcl_options, &state->context);
    });
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to initialize rcl");
  }
  throw_if_unparsed_ros_args(arguments.snapshot(), state->context.global_arguments);
  {
    std::lock_guard<std::mutex> guard(g_contexts_mutex);
    g_contexts.erase(
      std::remove_if(g_contexts.begin(), g_contexts.end(),
        [](const auto & weak) {return weak.expired();}), g_contexts.end());
    g_contexts.push_back(state);
  }
  std::atomic_store(&state_, std::move(state));
}

Context::Context(const Context & other)
: Destroyable(other), std::enable_shared_from_this<Context>(),
  state_(std::atomic_load(&other.state_))
{
}

std::shared_ptr<rcl_context_t>
Context::rcl_handle() const
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed context");
  }
  return std::shared_ptr<rcl_context_t>(state, &state->context);
}

rcl_context_t *
Context::rcl_ptr() const
{
  // Existing native callers hold a Destroyable lease or their own Context copy.
  return rcl_handle().get();
}

void
Context::destroy()
{
  std::atomic_store(&state_, std::shared_ptr<ContextState>());
}

size_t
Context::get_domain_id()
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed context");
  }
  size_t domain_id;
  const auto ret = native_context_call(state->mutex, [&]() {
      return rcl_context_get_domain_id(&state->context, &domain_id);
    });
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get domain id");
  }
  return domain_id;
}

bool
Context::ok()
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    return false;
  }
  return native_context_call(state->mutex, [&state]() {
      return rcl_context_is_valid(&state->context);
    });
}

void
Context::shutdown()
{
  auto state = std::atomic_load(&state_);
  if (!state) {
    throw InvalidHandle("cannot use a destroyed context");
  }
  const auto ret = native_context_call(state->mutex, [&state]() {
      return rcl_shutdown(&state->context);
    });
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to shutdown");
  }
  {
    std::lock_guard<std::mutex> guard(g_contexts_mutex);
    const std::weak_ptr<ContextState> target = state;
    g_contexts.erase(
      std::remove_if(g_contexts.begin(), g_contexts.end(),
        [&target](const auto & weak) {
          return weak.expired() ||
                 (!weak.owner_before(target) && !target.owner_before(weak));
        }),
      g_contexts.end());
  }
}

void define_context(py::object module)
{
  py::class_<Context, Destroyable, std::shared_ptr<Context>>(module, "Context")
  .def(py::init<py::list, size_t>())
  .def_property_readonly(
    "pointer", [](const Context & context) {
      return reinterpret_cast<size_t>(context.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_domain_id", &Context::get_domain_id,
    "Retrieves domain id from init_options of context.")
  .def(
    "ok", &Context::ok,
    "Status of the the client library")
  .def(
    "shutdown", &Context::shutdown,
    "Shutdown context");
}

}  // namespace rclpy
