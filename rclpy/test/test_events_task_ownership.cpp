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
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "events_executor/events_queue.hpp"
#include "events_executor/queued_task_ref.hpp"

namespace py = pybind11;
using rclpy::events_executor::EventsQueue;
using rclpy::events_executor::QueuedTaskRef;

void require(bool value, const char * message)
{
  if (!value) {
    throw std::runtime_error(message);
  }
}

int main()
{
  try {
    py::scoped_interpreter interpreter;
    auto sys = py::module_::import("sys");
    require(!py::cast<bool>(sys.attr("_is_gil_enabled")()), "natural GIL OFF required");
    require(sys.attr("flags").attr("gil").is_none(), "GIL override found");
    py::dict globals;
    py::exec("class Payload:\n    marker = 37\npayload = Payload()\n", globals);
    py::object payload = globals["payload"];
    globals.attr("pop")("payload");
    py::weakref observed(payload);
    const auto before = py::cast<py::ssize_t>(sys.attr("getrefcount")(observed));
    constexpr size_t thread_count = 8;
    constexpr size_t entries_per_thread = 128;
    std::vector<std::unique_ptr<EventsQueue>> queues;
    for (size_t i = 0; i < thread_count; ++i) {
      auto queue = std::make_unique<EventsQueue>();
      for (size_t j = 0; j < entries_per_thread; ++j) {
        auto reference = std::make_shared<QueuedTaskRef>(payload);
        queue->Enqueue([reference]() {});
      }
      queues.push_back(std::move(queue));
    }
    const auto queued = py::cast<py::ssize_t>(sys.attr("getrefcount")(observed));
    require(
      queued == before + static_cast<py::ssize_t>(thread_count * entries_per_thread),
      "queued weak owners missing");
    payload = py::none();
    require(observed().is_none(), "native queue unexpectedly retains payload");
    std::atomic<size_t> unattached{0};
    std::vector<std::thread> workers;
    {
      py::gil_scoped_release released;
      for (size_t i = 0; i < thread_count; ++i) {
        workers.emplace_back([&, i]() {
          if (PyThreadState_GetUnchecked() == nullptr) {
            unattached.fetch_add(1);
          }
          queues[i].reset();
          if (PyThreadState_GetUnchecked() == nullptr) {
            unattached.fetch_add(1);
          }
        });
      }
      for (auto & worker : workers) {
        worker.join();
      }
    }
    require(unattached == 2 * thread_count, "foreign workers retain a thread state");
    const auto after = py::cast<py::ssize_t>(sys.attr("getrefcount")(observed));
    require(after == before, "abandoned queue weak-reference owners leaked");
    require(!py::cast<bool>(sys.attr("_is_gil_enabled")()), "GIL enabled during component test");
    std::cout << "PASS real queued weak owners=1024 foreign cleanup threads=8 "
              << "payload collected=true weak_refcount_restored=true natural_GIL_OFF=true\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
