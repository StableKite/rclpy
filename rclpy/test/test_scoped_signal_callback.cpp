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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

#include "signal_handler.hpp"

namespace py = pybind11;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_copy_and_reentry()
{
  size_t calls = 0;
  {
    auto owner = std::make_unique<rclpy::ScopedSignalCallback>([&]() {++calls;});
    std::optional<rclpy::ScopedSignalCallback> alias(*owner);
    owner.reset();
    rclpy::detail::invoke_signal_callbacks();
    require(calls == 1, "copy did not retain the registration");
    alias.reset();
    rclpy::detail::invoke_signal_callbacks();
    require(calls == 1, "closed registration was invoked");
  }
  size_t first = 0;
  size_t next = 0;
  std::unique_ptr<rclpy::ScopedSignalCallback> owner;
  std::unique_ptr<rclpy::ScopedSignalCallback> added;
  owner = std::make_unique<rclpy::ScopedSignalCallback>([&]() {
      ++first;
      owner.reset();
      added = std::make_unique<rclpy::ScopedSignalCallback>([&]() {++next;});
    });
  rclpy::detail::invoke_signal_callbacks();
  require(first == 1 && next == 0, "dispatch snapshot changed during callback");
  rclpy::detail::invoke_signal_callbacks();
  require(first == 1 && next == 1, "reentrant registration did not run next time");
}

void test_close_cooperates_with_python_gc()
{
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false;
  bool released = false;
  std::atomic<bool> closing{false};
  std::atomic<bool> closed{false};
  std::atomic<bool> late{false};
  std::atomic<bool> gc_error{false};
  auto owner = std::make_unique<rclpy::ScopedSignalCallback>([&]() {
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      cv.notify_all();
      cv.wait(lock, [&]() {return released;});
      late.store(closed.load());
    });
  std::thread dispatcher([]() {rclpy::detail::invoke_signal_callbacks();});
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(5), [&]() {return entered;})) {
      released = true;
      cv.notify_all();
      lock.unlock();
      dispatcher.join();
      throw std::runtime_error("native callback did not start");
    }
  }
  std::thread collector([&]() {
      while (!closing.load()) {
        std::this_thread::yield();
      }
      {
        py::gil_scoped_acquire attach;
        if (PyGC_Collect() < 0) {
          gc_error.store(true);
          PyErr_Clear();
        }
      }
      std::lock_guard<std::mutex> lock(mutex);
      released = true;
      cv.notify_all();
    });
  closing.store(true);
  // The destructor must detach its waiting Python state; otherwise the
  // collector cannot stop the interpreter's threads to complete collection.
  owner.reset();
  closed.store(true);
  {
    py::gil_scoped_release release_python;
    dispatcher.join();
    collector.join();
  }
  require(!late.load(), "close returned before the active invocation ended");
  require(!gc_error.load(), "Python collection failed during native cleanup");
}

int main()
{
  py::scoped_interpreter interpreter;
  try {
    py::exec("import sys\nassert not sys._is_gil_enabled()\nassert sys.flags.gil is None");
    for (size_t i = 0; i < 32; ++i) {
      test_copy_and_reentry();
      test_close_cooperates_with_python_gc();
    }
    std::cout << "natural_GIL_OFF: 32 copy/reentry/drain/GC rounds passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
