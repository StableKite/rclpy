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
#include <rcl/timer.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "clock.hpp"
#include "context.hpp"
#include "timer.hpp"

namespace py = pybind11;

struct ResetWait
{
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false;
  bool released = false;
  std::atomic<bool> failed{false};
};

void blocking_callback(const void * data, size_t) noexcept
{
  auto & state = *const_cast<ResetWait *>(static_cast<const ResetWait *>(data));
  try {
    std::unique_lock<std::mutex> lock(state.mutex);
    state.entered = true;
    state.cv.notify_all();
    if (!state.cv.wait_for(lock, std::chrono::seconds(2), [&]() {return state.released;})) {
      state.failed.store(true);
    }
  } catch (...) {
    state.failed.store(true);
  }
}

void check_reset_gc(rclpy::Timer & timer)
{
  auto owner = timer.rcl_handle();
  ResetWait state;
  if (rcl_timer_set_on_reset_callback(owner.get(), blocking_callback, &state) != RCL_RET_OK) {
    throw std::runtime_error("could not register the SDK reset callback");
  }
  std::thread holder([&]() {
      if (rcl_timer_reset(owner.get()) != RCL_RET_OK) {
        state.failed.store(true);
      }
    });
  {
    py::gil_scoped_release detach;
    std::unique_lock<std::mutex> lock(state.mutex);
    if (!state.cv.wait_for(lock, std::chrono::seconds(5), [&]() {return state.entered;})) {
      state.failed.store(true);
    }
  }
  std::atomic<bool> collected{false};
  std::thread collector([&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      {
        py::gil_scoped_acquire attach;
        if (PyGC_Collect() < 0) {
          state.failed.store(true);
          PyErr_Clear();
        }
        collected.store(true);
      }
      std::lock_guard<std::mutex> lock(state.mutex);
      state.released = true;
      state.cv.notify_all();
    });
  try {
    timer.reset_timer();
  } catch (...) {
    state.failed.store(true);
  }
  {
    py::gil_scoped_release detach;
    collector.join();
    holder.join();
  }
  if (rcl_timer_set_on_reset_callback(owner.get(), nullptr, nullptr) != RCL_RET_OK) {
    throw std::runtime_error("could not drain the SDK reset callback");
  }
  if (state.failed.load() || !collected.load()) {
    throw std::runtime_error("native timer reset wait blocked Python garbage collection");
  }
}

int main()
{
  py::scoped_interpreter interpreter;
  try {
    py::exec("import sys\nassert not sys._is_gil_enabled() and sys.flags.gil is None");
    rclpy::Context context(py::list(), 219);
    rclpy::Clock clock(RCL_STEADY_TIME);
    rclpy::Timer timer(clock, context, 1000000000, true);
    for (size_t round = 0; round < 32; ++round) {
      check_reset_gc(timer);
    }
    context.shutdown();
    py::exec("assert not sys._is_gil_enabled() and sys.flags.gil is None");
    std::cout << "natural_GIL_OFF: 32 real SDK timer reset callback/GC rounds passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
