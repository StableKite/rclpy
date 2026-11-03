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
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "logging.hpp"

void test_waiting_guard_cooperates_with_gc()
{
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false;
  bool released = false;
  std::atomic<bool> collected{false};
  std::atomic<bool> gc_error{false};
  std::thread holder([&]() {
      rclpy::LoggingGuard logging;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      cv.notify_all();
      cv.wait(lock, [&]() {return released;});
    });
  {
    py::gil_scoped_release release_python;
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&]() {return entered;});
  }
  std::thread collector([&]() {
      {
        py::gil_scoped_acquire attach;
        if (PyGC_Collect() < 0) {
          gc_error.store(true);
          PyErr_Clear();
        }
        collected.store(true);
      }
      std::lock_guard<std::mutex> lock(mutex);
      released = true;
      cv.notify_all();
    });
  {
    // This wait must detach Python state for the collector to stop this
    // thread. Unlock must also precede reattachment when the guard exits.
    rclpy::LoggingGuard logging;
  }
  {
    py::gil_scoped_release release_python;
    collector.join();
    holder.join();
  }
  if (!collected.load() || gc_error.load()) {
    throw std::runtime_error("collection did not complete while waiting for the native logging guard");
  }
}

int main()
{
  py::scoped_interpreter interpreter;
  try {
    py::exec("import sys\nassert not sys._is_gil_enabled()\nassert sys.flags.gil is None");
    for (size_t i = 0; i < 32; ++i) {
      test_waiting_guard_cooperates_with_gc();
    }
    std::cout << "natural_GIL_OFF: 32 native logging guard/GC rounds passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
