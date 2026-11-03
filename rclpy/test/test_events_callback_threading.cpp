// Copyright 2026 StableKite contributors
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

#include <pybind11/embed.h>
#include <rcl/time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "destroyable.hpp"
#include "time_point.hpp"
#include "events_executor/events_queue.hpp"
#include "events_executor/rcl_support.hpp"

namespace py = pybind11;
namespace events = rclpy::events_executor;

// This embeds only the already ported Clock/Destroyable components, not _rclpy.
PYBIND11_EMBEDDED_MODULE(sk_events_callback_native, module, py::mod_gil_not_used())
{
  rclpy::define_destroyable(module);
  rclpy::define_time_point(module);
  rclpy::define_clock(module);
}

void check_generation()
{
  auto clock = std::make_shared<rclpy::Clock>(RCL_STEADY_TIME);
  auto owner = py::cast(clock);
  const void * key = clock->rcl_ptr();
  events::EventsQueue queue;
  events::RclCallbackManager manager(&queue);
  size_t removed_calls = 0;
  size_t replacement_calls = 0;
  for (size_t round = 0; round < 128; ++round) {
    auto old_scope = std::make_shared<events::ScopedWith>(owner);
    const void * old_data = manager.MakeCallback(key, [&](size_t count) {
          clock->get_now();
          removed_calls += count;
        }, old_scope);
    events::RclEventCallbackTrampoline(old_data, 1);
    // The SDK is no longer using old_data before removal, as required by the API.
    manager.RemoveCallback(key);
    old_scope.reset();
    auto new_scope = std::make_shared<events::ScopedWith>(owner);
    const void * new_data = manager.MakeCallback(key, [&](size_t count) {
          clock->get_now();
          replacement_calls += count;
        }, new_scope);
    events::RclEventCallbackTrampoline(new_data, 2);
    queue.Run(std::chrono::steady_clock::now());
    manager.RemoveCallback(key);
    new_scope.reset();
    if (removed_calls != 0 || replacement_calls != (round + 1) * 2) {
      throw std::runtime_error("removed registration dispatched after same-key replacement: old=" +
              std::to_string(removed_calls) + " new=" + std::to_string(replacement_calls));
    }
  }
  std::cout << "generation: 128 real Clock keys reused; removed=0 replacement=256\n";
}

void check_retired_manager()
{
  auto clock = std::make_shared<rclpy::Clock>(RCL_STEADY_TIME);
  auto owner = py::cast(clock);
  events::EventsQueue queue;
  size_t calls = 0;
  {
    events::RclCallbackManager manager(&queue);
    auto scope = std::make_shared<events::ScopedWith>(owner);
    const void * data = manager.MakeCallback(clock->rcl_ptr(), [&](size_t count) {
          calls += count;
        }, scope);
    events::RclEventCallbackTrampoline(data, 1);
    manager.RemoveCallback(clock->rcl_ptr());
  }
  queue.Run(std::chrono::steady_clock::now());
  if (calls != 0) {
    throw std::runtime_error("retired callback manager still dispatched a removed callback");
  }
  std::cout << "retired manager: pending removed event safely drained\n";
}

void check_inflight_lease()
{
  auto clock = std::make_shared<rclpy::Clock>(RCL_STEADY_TIME);
  auto owner = py::cast(clock);
  const void * key = clock->rcl_ptr();
  events::EventsQueue queue;
  events::RclCallbackManager manager(&queue);
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false;
  bool released = false;
  std::atomic<bool> failed{false};
  std::atomic<size_t> calls{0};
  auto scope = std::make_shared<events::ScopedWith>(owner);
  const void * data = manager.MakeCallback(key, [&](size_t count) {
        {
          std::unique_lock<std::mutex> lock(mutex);
          entered = true;
          cv.notify_all();
          if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return released;})) {
            failed.store(true);
            return;
          }
        }
        // The Python owner has requested destruction, but this in-flight event
        // must retain the actual native Clock lease until it has finished.
        clock->get_now();
        calls.fetch_add(count);
      }, scope);
  scope.reset();
  events::RclEventCallbackTrampoline(data, 3);
  std::thread consumer([&]() {
      try {
        queue.Run(std::chrono::steady_clock::now());
      } catch (const std::exception & error) {
        py::gil_scoped_acquire attached;
        std::cerr << "in-flight callback: " << error.what() << '\n';
        failed.store(true);
      }
    });
  {
    py::gil_scoped_release detached;
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return entered;})) {
      failed.store(true);
    }
  }
  manager.RemoveCallback(key);
  clock->destroy_when_not_in_use();
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    cv.notify_all();
  }
  {
    py::gil_scoped_release detached;
    consumer.join();
  }
  if (failed.load() || calls.load() != 3) {
    throw std::runtime_error("removal destroyed an active callback's native Clock lease");
  }
  std::cout << "in-flight: real Clock survives concurrent removal and destroy request\n";
}

void check_native_threads()
{
  events::EventsQueue queue;
  events::RclCallbackManager manager(&queue);
  std::vector<std::shared_ptr<rclpy::Clock>> clocks;
  std::vector<py::object> owners;
  for (size_t worker = 0; worker < 8; ++worker) {
    clocks.push_back(std::make_shared<rclpy::Clock>(RCL_STEADY_TIME));
    owners.push_back(py::cast(clocks.back()));
  }
  std::vector<Py_ssize_t> reference_counts;
  for (const auto & owner : owners) {
    reference_counts.push_back(owner.ref_count());
  }
  std::atomic<bool> failed{false};
  std::atomic<size_t> dispatched{0};
  std::vector<std::thread> producers;
  std::thread consumer([&]() {
      try {
        queue.Run(std::chrono::steady_clock::now() + std::chrono::seconds(15));
      } catch (const std::exception & error) {
        py::gil_scoped_acquire attached;
        std::cerr << "queue consumer: " << error.what() << '\n';
        failed.store(true);
      }
    });
  for (size_t worker = 0; worker < 8; ++worker) {
    producers.emplace_back([&, worker]() {
        try {
          for (size_t round = 0; round < 128; ++round) {
            // Real Python context entry/ref ownership and exit happen on a
            // native thread with no prior Python thread state.
            auto scope = std::make_shared<events::ScopedWith>(owners[worker]);
            const void * key = clocks[worker]->rcl_ptr();
            const void * data = manager.MakeCallback(key, [&, worker](size_t count) {
                  clocks[worker]->get_now();
                  dispatched.fetch_add(count);
                }, scope);
            events::RclEventCallbackTrampoline(data, 1);
            events::RclEventCallbackTrampoline(data, 1);
            manager.RemoveCallback(key);
          }
        } catch (const std::exception & error) {
          py::gil_scoped_acquire attached;
          std::cerr << "native producer: " << error.what() << '\n';
          failed.store(true);
        }
      });
  }
  {
    py::gil_scoped_release detached;
    for (auto & producer : producers) {
      producer.join();
    }
    queue.Enqueue([&]() {queue.Stop();});
    consumer.join();
  }
  if (failed.load() || dispatched.load() > 2048) {
    throw std::runtime_error("native callback registration/dispatch/retirement failed");
  }
  for (size_t worker = 0; worker < owners.size(); ++worker) {
    if (owners[worker].ref_count() != reference_counts[worker]) {
      throw std::runtime_error("native callback retirement leaked its Python Clock owner");
    }
  }
  std::cout << "native threads: 8 producers/1024 real Clock leases/2048 events; accepted=" <<
    dispatched.load() << " (events retired before dispatch are cancelled)\n";
}

void check_failed_entry()
{
  auto clock = std::make_shared<rclpy::Clock>(RCL_STEADY_TIME);
  auto owner = py::cast(clock);
  clock->destroy_when_not_in_use();
  const auto before = owner.ref_count();
  bool rejected = false;
  try {
    events::ScopedWith scope(owner);
  } catch (const py::error_already_set &) {
    rejected = true;
  }
  if (!rejected || owner.ref_count() != before) {
    throw std::runtime_error("failed actual Destroyable entry leaked its Python owner");
  }
  std::cout << "failed entry: real destroyed Clock rejected; owner refcount unchanged\n";
}

int main()
{
  py::scoped_interpreter interpreter;
  try {
    py::exec(
      "import sys\n"
      "assert not sys._is_gil_enabled() and sys.flags.gil is None\n"
      "unraisables = []\n"
      "original_unraisablehook = sys.unraisablehook\n"
      "def record_unraisable(error):\n"
      "    unraisables.append(error.exc_type)\n"
      "    original_unraisablehook(error)\n"
      "sys.unraisablehook = record_unraisable\n");
    py::module_::import("sk_events_callback_native");
    check_generation();
    check_retired_manager();
    check_inflight_lease();
    check_native_threads();
    check_failed_entry();
    py::exec(
      "assert not sys._is_gil_enabled() and sys.flags.gil is None\n"
      "assert not unraisables, unraisables\n"
      "sys.unraisablehook = original_unraisablehook\n");
    std::cout << "natural_GIL_OFF: actual EventsQueue/RclCallbackManager/Clock passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
