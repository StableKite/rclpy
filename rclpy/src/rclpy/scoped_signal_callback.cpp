// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "signal_handler.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace rclpy
{
// A registration owner is separate from dispatcher snapshots: destroying the
// last owner must disable and drain a callback even if a snapshot still exists.
class SignalCallbackState
{
public:
  explicit SignalCallbackState(std::function<void()> callback)
  : callback_(std::move(callback)) {}

  void disable()
  {
    // A callback runs on the native deferred worker. It may need Python state
    // before completing, so a Python owner must not retain its state while
    // waiting for that invocation to finish.
    std::optional<py::gil_scoped_release> release;
#if PY_VERSION_HEX >= 0x030D0000
    if (Py_IsInitialized() && PyThreadState_GetUnchecked() != nullptr) {
#else
    if (Py_IsInitialized() && PyGILState_Check()) {
#endif
      release.emplace();
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    enabled_ = false;
    // Keep the function itself intact: it may be unregistering its own owner
    // while its call operator is still on this thread's stack.
  }

  void invoke()
  {
    // Only the deferred dispatcher invokes callbacks. A per-entry recursive
    // guard preserves synchronous destruction and same-thread unregister,
    // without holding the registry mutex across callback code.
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (enabled_) {
      callback_();
    }
  }

private:
  std::recursive_mutex mutex_;
  bool enabled_ = true;
  std::function<void()> callback_;
};

std::vector<std::weak_ptr<SignalCallbackState>> g_callbacks;
std::mutex g_callback_mutex;
class ScopedSignalCallback::Impl
{
public:
  explicit Impl(std::function<void()> callback)
  : state_(std::make_shared<SignalCallbackState>(std::move(callback)))
  {
    std::lock_guard<std::mutex> lock(g_callback_mutex);
    g_callbacks.erase(
      std::remove_if(g_callbacks.begin(), g_callbacks.end(),
        [](const auto & weak) {return weak.expired();}), g_callbacks.end());
    g_callbacks.push_back(state_);
  }

  ~Impl()
  {
    state_->disable();
  }

private:
  std::shared_ptr<SignalCallbackState> state_;
};

ScopedSignalCallback::ScopedSignalCallback(std::function<void()> cb)
: impl_(std::make_shared<Impl>(cb))
{
}

ScopedSignalCallback::~ScopedSignalCallback() {}


namespace detail
{
void invoke_signal_callbacks()
{
  std::vector<std::shared_ptr<rclpy::SignalCallbackState>> callbacks;
  {
    std::lock_guard<std::mutex> lock(rclpy::g_callback_mutex);
    callbacks.reserve(rclpy::g_callbacks.size());
    for (auto it = rclpy::g_callbacks.begin(); it != rclpy::g_callbacks.end(); ) {
      if (auto callback = it->lock()) {
        callbacks.push_back(std::move(callback));
        ++it;
      } else {
        it = rclpy::g_callbacks.erase(it);
      }
    }
  }
  for (const auto & callback : callbacks) {
    callback->invoke();
  }
}

}  // namespace detail
}  // namespace rclpy
