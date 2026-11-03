// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
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
#include "events_executor/rcl_support.hpp"

#include <utility>

namespace py = pybind11;

namespace rclpy
{
namespace events_executor
{

extern "C" void RclEventCallbackTrampoline(const void * user_data, size_t number_of_events)
{
  const auto cb = reinterpret_cast<const std::function<void(size_t)> *>(user_data);
  (*cb)(number_of_events);
}

RclCallbackManager::RclCallbackManager(EventsQueue * events_queue)
: events_queue_(events_queue) {}

RclCallbackManager::~RclCallbackManager()
{
  // Should not still have any callbacks registered when we exit, because otherwise RCL can call
  // pointers that will no longer be valid.  We can't throw an exception here, but we can explode.
  bool callbacks_remaining;
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    callbacks_remaining = !owned_cbs_.empty();
  }
  if (callbacks_remaining) {
    py::gil_scoped_acquire gil_acquire;
    py::print("Destroying callback manager with callbacks remaining");
    ::abort();
  }
}

const void * RclCallbackManager::MakeCallback(
  const void * key, std::function<void(size_t)> callback, std::shared_ptr<ScopedWith> with)
{
  // We don't support replacing an existing callback with a new one, because it gets tricky making
  // sure we don't delete an old callback while the middleware still holds a pointer to it.
  CbEntry new_entry;
  new_entry.state = std::make_shared<CallbackState>();
  new_entry.state->invocation = std::make_shared<Invocation>(
    Invocation{std::move(callback), std::move(with)});
  auto state = new_entry.state;
  auto queue = events_queue_;
  new_entry.cb =
    std::make_unique<std::function<void(size_t)>>([queue, state](size_t number_of_events) {
        queue->Enqueue([state, number_of_events]() {
          std::shared_ptr<Invocation> invocation;
          {
            std::lock_guard<std::mutex> lock(state->mutex);
            invocation = state->invocation;
          }
          if (invocation) {
            // No manager pointer, map lookup, or native mutex may survive into
            // user code. Removal can retire this registration concurrently.
            invocation->callback(number_of_events);
          }
      });
    });
  const void * ret = new_entry.cb.get();
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    if (!owned_cbs_.try_emplace(key, std::move(new_entry)).second) {
      throw py::key_error("Attempt to replace existing callback");
    }
  }
  return ret;
}

int RclCallbackManager::TraversePythonOwners(visitproc visit, void * arg) const
{
  // Registration and retirement happen on the attached executor thread. GC
  // stops that thread; middleware only enqueues work. Do not acquire a Python
  // or native registry lock from tp_traverse while mutators are stopped.
  for (auto current = owned_cbs_.cbegin(); current != owned_cbs_.cend(); ++current) {
    const auto * invocation = current->second.state->invocation.get();
    if (!invocation || !invocation->with) {
      continue;
    }
    const auto * owner = invocation->with.get();
    bool already_visited = false;
    // One waitable can share its ScopedWith across multiple registrations.
    // It owns one Python reference, so visit that reference exactly once.
    // This allocation-free walk also keeps tp_traverse free of allocation.
    for (auto previous = owned_cbs_.cbegin(); previous != current; ++previous) {
      const auto * earlier = previous->second.state->invocation.get();
      if (earlier && earlier->with.get() == owner) {
        already_visited = true;
        break;
      }
    }
    if (!already_visited) {
      const int result = owner->TraversePythonOwner(visit, arg);
      if (result != 0) {
        return result;
      }
    }
  }
  return 0;
}

void RclCallbackManager::RemoveCallback(const void * key)
{
  CbEntry removed;
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    auto it = owned_cbs_.find(key);
    if (it == owned_cbs_.end()) {
      throw py::key_error("Attempt to remove nonexistent callback");
    }
    removed = std::move(it->second);
    owned_cbs_.erase(it);
  }
  std::shared_ptr<Invocation> retired;
  {
    std::lock_guard<std::mutex> lock(removed.state->mutex);
    retired = std::move(removed.state->invocation);
  }
  // Both the Python context exit and callback capture destruction run outside
  // native mutexes. An in-flight event retains its own invocation/owner lease.
}

}  // namespace events_executor
}  // namespace rclpy
