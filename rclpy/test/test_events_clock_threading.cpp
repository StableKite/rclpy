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
#include <rcl/timer.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "context.hpp"
#include "timer.hpp"
#include "events_executor/events_queue.hpp"
#include "events_executor/timers_manager.hpp"

namespace py = pybind11;
namespace events = rclpy::events_executor;

PYBIND11_EMBEDDED_MODULE(sk_events_clock_native, module, py::mod_gil_not_used())
{
  py::enum_<rcl_clock_change_t>(module, "ClockChange")
  .value("ROS_TIME_NO_CHANGE", RCL_ROS_TIME_NO_CHANGE)
  .value("ROS_TIME_ACTIVATED", RCL_ROS_TIME_ACTIVATED)
  .value("ROS_TIME_DEACTIVATED", RCL_ROS_TIME_DEACTIVATED)
  .value("SYSTEM_TIME_NO_CHANGE", RCL_SYSTEM_TIME_NO_CHANGE);
}

void check_clock_registration_guard(rclpy::Clock & clock, rclpy::Timer & timer)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  std::mutex mutex;
  std::condition_variable cv;
  bool begin_add = false;
  bool add_attempted = false;
  bool added = false;
  bool begin_remove = false;
  bool remove_attempted = false;
  bool removed = false;
  std::atomic<bool> failed{false};
  std::thread worker([&]() {
      try {
        {
          std::unique_lock<std::mutex> lock(mutex);
          if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return begin_add;})) {
            throw std::runtime_error("add was never requested");
          }
          add_attempted = true;
          cv.notify_all();
        }
        manager.AddTimer(timer.rcl_ptr(), []() {});
        {
          std::unique_lock<std::mutex> lock(mutex);
          added = true;
          cv.notify_all();
          if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return begin_remove;})) {
            throw std::runtime_error("remove was never requested");
          }
          remove_attempted = true;
          cv.notify_all();
        }
        manager.RemoveTimer(timer.rcl_ptr());
        {
          std::lock_guard<std::mutex> lock(mutex);
          removed = true;
          cv.notify_all();
        }
      } catch (const std::exception & error) {
        py::gil_scoped_acquire attached;
        std::cerr << "clock registration worker: " << error.what() << '\n';
        failed.store(true);
        cv.notify_all();
      }
    });
  bool add_bypassed_guard = false;
  bool remove_bypassed_guard = false;
  {
    auto guard = clock.mutation();
    {
      std::lock_guard<std::mutex> lock(mutex);
      begin_add = true;
      cv.notify_all();
    }
    py::gil_scoped_release detached;
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {
        return add_attempted || failed.load();
      }))
    {
      failed.store(true);
    }
    add_bypassed_guard = cv.wait_for(lock, std::chrono::milliseconds(250), [&]() {
          return added || failed.load();
        }) && added;
  }
  {
    py::gil_scoped_release detached;
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return added || failed.load();})) {
      failed.store(true);
    }
  }
  {
    auto guard = clock.mutation();
    {
      std::lock_guard<std::mutex> lock(mutex);
      begin_remove = true;
      cv.notify_all();
    }
    py::gil_scoped_release detached;
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {
        return remove_attempted || failed.load();
      }))
    {
      failed.store(true);
    }
    remove_bypassed_guard = cv.wait_for(lock, std::chrono::milliseconds(250), [&]() {
          return removed || failed.load();
        }) && removed;
  }
  {
    py::gil_scoped_release detached;
    worker.join();
  }
  if (failed.load() || !added || !removed) {
    throw std::runtime_error("actual timer registration did not finish");
  }
  std::cout << "clock guard: add_bypassed=" << add_bypassed_guard <<
    " remove_bypassed=" << remove_bypassed_guard << '\n';
  if (add_bypassed_guard || remove_bypassed_guard) {
    throw std::runtime_error("EventsExecutor mutated the real SDK clock callback array outside ClockMutation");
  }
  queue.Run(std::chrono::steady_clock::now());
}

void check_reentrant_registration(rclpy::Clock & clock, rclpy::Timer & timer)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  auto observer_type = py::module_::import("types").attr("new_class")("NativeClockObserver");
  auto observer = observer_type();
  auto added_observer = observer_type();
  observer.attr("_pre_callback") = py::none();
  added_observer.attr("_pre_callback") = py::none();
  const auto initial_count = clock.rcl_ptr()->num_jump_callbacks;
  size_t phase = 0;
  size_t added_calls = 0;
  bool timer_managed = false;
  bool python_managed = false;
  bool observer_managed = false;
  added_observer.attr("_post_callback") = py::cpp_function([&](py::dict) {++added_calls;});
  observer.attr("_post_callback") = py::cpp_function([&](py::dict) {
        const auto before = clock.rcl_ptr()->num_jump_callbacks;
        if (phase == 0) {
          manager.AddTimer(timer.rcl_ptr(), []() {});
          timer_managed = true;
          clock.add_clock_callback(added_observer, true, 1, -1);
          python_managed = true;
        } else if (phase == 2) {
          manager.RemoveTimer(timer.rcl_ptr());
          timer_managed = false;
          clock.remove_clock_callback(added_observer);
          python_managed = false;
        }
        if (clock.rcl_ptr()->num_jump_callbacks != before) {
          throw std::runtime_error("reentrant registration changed the SDK callback array during traversal");
        }
        ++phase;
      });
  auto cleanup = [&]() {
      if (timer_managed) {
        manager.RemoveTimer(timer.rcl_ptr());
        timer_managed = false;
      }
      if (python_managed) {
        clock.remove_clock_callback(added_observer);
        python_managed = false;
      }
      if (observer_managed) {
        clock.remove_clock_callback(observer);
        observer_managed = false;
      }
    };
  try {
    clock.add_clock_callback(observer, true, 1, -1);
    observer_managed = true;
    clock.set_ros_time_override({1000000100, RCL_ROS_TIME});
    if (phase != 1 || added_calls != 0 || clock.rcl_ptr()->num_jump_callbacks != initial_count + 3) {
      throw std::runtime_error("pending native/Python registrations were not established after traversal");
    }
    clock.set_ros_time_override({1000000200, RCL_ROS_TIME});
    queue.Run(std::chrono::steady_clock::now());
    if (phase != 2 || added_calls != 1) {
      throw std::runtime_error("deferred Python callback did not receive the next real clock jump");
    }
    clock.set_ros_time_override({1000000300, RCL_ROS_TIME});
    queue.Run(std::chrono::steady_clock::now());
    if (phase != 3 || added_calls != 1 || clock.rcl_ptr()->num_jump_callbacks != initial_count + 1) {
      throw std::runtime_error("reentrantly removed native/Python registrations remained active");
    }
    cleanup();
  } catch (...) {
    cleanup();
    throw;
  }
  if (clock.rcl_ptr()->num_jump_callbacks != initial_count) {
    throw std::runtime_error("reentrant registration leaked real SDK callback userdata");
  }
  std::cout << "reentrant: native+Python add/remove deferred until SDK traversal finishes\n";
}

void check_ready_delivery(rclpy::Clock & clock, rclpy::Timer & timer)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  clock.set_ros_time_override({1000000000, RCL_ROS_TIME});
  timer.reset_timer();
  size_t ready = 0;
  manager.AddTimer(timer.rcl_ptr(), [&]() {++ready;});
  int64_t next_before = 0;
  int64_t next_after_jump = 0;
  int64_t next_after_dispatch = 0;
  const auto before_status = rcl_timer_get_next_call_time(timer.rcl_ptr(), &next_before);
  clock.set_ros_time_override({3000000000, RCL_ROS_TIME});
  const auto jump_status = rcl_timer_get_next_call_time(timer.rcl_ptr(), &next_after_jump);
  const auto now = clock.get_now().nanoseconds;
  queue.Run(std::chrono::steady_clock::now());
  const auto dispatch_status = rcl_timer_get_next_call_time(timer.rcl_ptr(), &next_after_dispatch);
  std::cout << "delivery diagnostic: next_before=" << next_before << " after_jump=" <<
    next_after_jump << " now=" << now << " after_dispatch=" << next_after_dispatch <<
    " ready=" << ready << " status=" << before_status << ',' << jump_status << ',' <<
    dispatch_status << '\n';
  manager.RemoveTimer(timer.rcl_ptr());
  timer.cancel_timer();
  if (ready != 1) {
    throw std::runtime_error("actual ROS time jump did not deliver exactly one native ready timer: " +
            std::to_string(ready));
  }
  std::cout << "delivery: real ROS time jump -> native timer call -> ready callback exactly once\n";
}

void check_threaded_clock_updates(rclpy::Clock & clock, rclpy::Timer & timer)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  const auto initial_count = clock.rcl_ptr()->num_jump_callbacks;
  std::atomic<bool> start{false};
  std::atomic<bool> failed{false};
  std::atomic<int64_t> next_time{4000000000};
  std::vector<std::thread> updaters;
  for (size_t index = 0; index < 4; ++index) {
    updaters.emplace_back([&]() {
        while (!start.load()) {
          std::this_thread::yield();
        }
        try {
          for (size_t step = 0; step < 256; ++step) {
            clock.set_ros_time_override({next_time.fetch_add(1000), RCL_ROS_TIME});
          }
        } catch (const std::exception & error) {
          py::gil_scoped_acquire attached;
          std::cerr << "native clock updater: " << error.what() << '\n';
          failed.store(true);
        }
      });
  }
  std::thread registrar([&]() {
      while (!start.load()) {
        std::this_thread::yield();
      }
      try {
        for (size_t round = 0; round < 128; ++round) {
          manager.AddTimer(timer.rcl_ptr(), []() {});
          const auto reset = rcl_timer_reset(timer.rcl_ptr());
          manager.RemoveTimer(timer.rcl_ptr());
          if (reset != RCL_RET_OK) {
            throw std::runtime_error("actual SDK timer reset failed");
          }
        }
      } catch (const std::exception & error) {
        py::gil_scoped_acquire attached;
        std::cerr << "native clock registrar: " << error.what() << '\n';
        failed.store(true);
      }
    });
  start.store(true);
  {
    py::gil_scoped_release detached;
    for (auto & updater : updaters) {
      updater.join();
    }
    registrar.join();
  }
  queue.Run(std::chrono::steady_clock::now());
  timer.cancel_timer();
  if (failed.load() || clock.rcl_ptr()->num_jump_callbacks != initial_count) {
    throw std::runtime_error("concurrent real ROS clock updates/registration/reset/retirement failed");
  }
  std::cout << "threads: 4 native clock setters/1024 jumps concurrent with128 registrations+SDK resets\n";
}

void check_failed_registration(rclpy::Clock & clock, rclpy::Timer & timer)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  auto native = clock.rcl_handle();
  struct Failure
  {
    rcl_allocator_t original;
    size_t original_bytes;
    bool failed = false;
  } failure{native->allocator, native->num_jump_callbacks * sizeof(rcl_jump_callback_info_t)};
  const auto initial_count = native->num_jump_callbacks;
  native->allocator.state = &failure;
  native->allocator.allocate = [](size_t size, void * data) {
      auto & state = *static_cast<Failure *>(data);
      return state.original.allocate(size, state.original.state);
    };
  native->allocator.zero_allocate = [](size_t count, size_t size, void * data) {
      auto & state = *static_cast<Failure *>(data);
      return state.original.zero_allocate(count, size, state.original.state);
    };
  native->allocator.deallocate = [](void * pointer, void * data) {
      auto & state = *static_cast<Failure *>(data);
      state.original.deallocate(pointer, state.original.state);
    };
  native->allocator.reallocate = [](void * pointer, size_t size, void * data) -> void * {
      auto & state = *static_cast<Failure *>(data);
      if (!state.failed && size > state.original_bytes) {
        state.failed = true;
        return nullptr;
      }
      return state.original.reallocate(pointer, size, state.original.state);
    };
  bool rejected = false;
  try {
    manager.AddTimer(timer.rcl_ptr(), []() {});
  } catch (const rclpy::RCLError &) {
    rejected = true;
  } catch (...) {
    native->allocator = failure.original;
    throw;
  }
  native->allocator = failure.original;
  if (!rejected || !failure.failed || native->num_jump_callbacks != initial_count) {
    throw std::runtime_error("failed actual SDK callback allocation did not roll back registration");
  }
  manager.AddTimer(timer.rcl_ptr(), []() {});
  manager.RemoveTimer(timer.rcl_ptr());
  if (native->num_jump_callbacks != initial_count) {
    throw std::runtime_error("registration retry retained SDK userdata");
  }
  std::cout << "allocation: actual SDK allocator failure propagates; exact rollback and retry passed\n";
}

void check_reused_timer_address(
  rclpy::Clock & clock, rclpy::Timer & anchor, rclpy::Context & context)
{
  events::EventsQueue queue;
  events::RclTimersManager manager(&queue);
  rcl_timer_t reused = rcl_get_zero_initialized_timer();
  bool initialized = false;
  bool managed = false;
  bool anchor_managed = false;
  size_t old_calls = 0;
  size_t new_calls = 0;
  auto context_owner = context.rcl_handle();
  const auto initial_count = clock.rcl_ptr()->num_jump_callbacks;
  auto initialize = [&](int64_t period) {
      auto guard = clock.mutation();
      if (rcl_timer_init2(&reused, guard.rcl_ptr(), context_owner.get(), period,
          nullptr, rcl_get_default_allocator(), true) != RCL_RET_OK)
      {
        throw std::runtime_error("real SDK raw timer initialization failed");
      }
      initialized = true;
    };
  auto finalize = [&]() {
      auto guard = clock.mutation();
      const auto ret = rcl_timer_fini(&reused);
      initialized = false;
      if (ret != RCL_RET_OK) {
        throw std::runtime_error("real SDK raw timer finalization failed");
      }
      reused = rcl_get_zero_initialized_timer();
    };
  auto cleanup = [&]() {
      if (managed) {
        manager.RemoveTimer(&reused);
        managed = false;
      }
      if (initialized) {
        finalize();
      }
      if (anchor_managed) {
        manager.RemoveTimer(anchor.rcl_ptr());
        anchor_managed = false;
      }
    };
  try {
    clock.set_ros_time_override({1000000000, RCL_ROS_TIME});
    manager.AddTimer(anchor.rcl_ptr(), []() {});
    anchor_managed = true;
    initialize(100);
    clock.set_ros_time_override({1000000100, RCL_ROS_TIME});
    manager.AddTimer(&reused, [&]() {++old_calls;});
    managed = true;
    manager.RemoveTimer(&reused);
    managed = false;
    finalize();
    // SDK-supported fini/init reuse of the same actual rcl_timer_t storage.
    initialize(1000000000);
    manager.AddTimer(&reused, [&]() {++new_calls;});
    managed = true;
    int64_t expected_next = 0;
    int64_t actual_next = 0;
    if (rcl_timer_get_next_call_time(&reused, &expected_next) != RCL_RET_OK) {
      throw std::runtime_error("could not read the newly initialized SDK timer");
    }
    queue.Run(std::chrono::steady_clock::now());
    const auto ret = rcl_timer_get_next_call_time(&reused, &actual_next);
    std::cout << "reused address: removed_calls=" << old_calls << " new_early_calls=" <<
      new_calls << " next_before=" << expected_next << " next_after=" << actual_next << '\n';
    cleanup();
    if (ret != RCL_RET_OK || old_calls || new_calls || expected_next != actual_next) {
      throw std::runtime_error("retired timer event dispatched a different live SDK timer at the same address");
    }
  } catch (...) {
    cleanup();
    throw;
  }
  if (clock.rcl_ptr()->num_jump_callbacks != initial_count) {
    throw std::runtime_error("same-address SDK timer retirement leaked callbacks");
  }
}

void check_foreign_sdk_clock(rclpy::Context & context)
{
  auto allocator = rcl_get_default_allocator();
  rcl_clock_t clock{};
  if (rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator) != RCL_RET_OK) {
    throw std::runtime_error("foreign native SDK clock initialization failed");
  }
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  auto context_owner = context.rcl_handle();
  if (rcl_timer_init2(&timer, &clock, context_owner.get(), 1000000000, nullptr,
      allocator, false) != RCL_RET_OK)
  {
    if (rcl_clock_fini(&clock) != RCL_RET_OK) {
      throw std::runtime_error("foreign native SDK clock rollback failed");
    }
    throw std::runtime_error("foreign native SDK timer initialization failed");
  }
  {
    events::EventsQueue queue;
    events::RclTimersManager manager(&queue);
    manager.AddTimer(&timer, []() {});
    const auto reset = rcl_timer_reset(&timer);
    manager.RemoveTimer(&timer);
    queue.Run(std::chrono::steady_clock::now());
    if (reset != RCL_RET_OK || clock.num_jump_callbacks != 0) {
      throw std::runtime_error("original foreign SDK clock/timer API regressed");
    }
  }
  const auto timer_ret = rcl_timer_fini(&timer);
  const auto clock_ret = rcl_clock_fini(&clock);
  if (timer_ret != RCL_RET_OK || clock_ret != RCL_RET_OK) {
    throw std::runtime_error("foreign native SDK resources did not finalize");
  }
  std::cout << "foreign SDK clock: original native init/register/reset/remove/fini API passed\n";
}

int main(int argc, char ** argv)
{
  py::scoped_interpreter interpreter;
  try {
    py::exec(
      "import sys\nassert not sys._is_gil_enabled() and sys.flags.gil is None\n"
      "unraisables = []\noriginal_unraisablehook = sys.unraisablehook\n"
      "def record_unraisable(error):\n"
      "    unraisables.append(error.exc_type)\n"
      "    original_unraisablehook(error)\n"
      "sys.unraisablehook = record_unraisable\n");
    py::module_::import("sk_events_clock_native");
    rclpy::Context context(py::list(), 219);
    {
      rclpy::Clock clock(RCL_ROS_TIME);
      clock.set_ros_time_override_is_enabled(true);
      clock.set_ros_time_override({1000000000, RCL_ROS_TIME});
      rclpy::Timer timer(clock, context, 1000000000, false);
      if (argc == 2 && std::string(argv[1]) == "--queue-generation") {
        check_reused_timer_address(clock, timer, context);
      } else if (argc == 1) {
        check_clock_registration_guard(clock, timer);
        check_reentrant_registration(clock, timer);
        check_ready_delivery(clock, timer);
        check_threaded_clock_updates(clock, timer);
        check_failed_registration(clock, timer);
        check_reused_timer_address(clock, timer, context);
        check_foreign_sdk_clock(context);
      } else {
        throw std::runtime_error("unrecognized native test arguments");
      }
    }
    context.shutdown();
    py::exec(
      "assert not sys._is_gil_enabled() and sys.flags.gil is None\n"
      "assert not unraisables, unraisables\nsys.unraisablehook = original_unraisablehook\n");
    std::cout << "natural_GIL_OFF: actual EventsExecutor SDK clock mutation serialization passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
