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
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include "events_executor/python_hasher.hpp"

namespace py = pybind11;
using EntityMap = std::unordered_map<py::handle, int,
    rclpy::events_executor::PythonHasher, rclpy::events_executor::PythonIdentityEqual>;

void check_distinct_entities(py::handle first, py::handle second)
{
  EntityMap entities;
  entities.emplace(first, 10);
  entities.emplace(second, 20);
  if (entities.size() != 2 || entities.at(first) != 10 || entities.at(second) != 20) {
    throw std::runtime_error("distinct entity identities were merged");
  }
  if (entities.erase(first) != 1 || entities.count(first) != 0 || entities.at(second) != 20) {
    throw std::runtime_error("removing one entity changed a different identity");
  }
  const auto unhashable = py::list();
  try {
    entities.emplace(unhashable, 30);
    throw std::runtime_error("an unhashable entity was accepted");
  } catch (const py::error_already_set & error) {
    if (!error.matches(PyExc_TypeError)) {
      throw;
    }
  }
  if (entities.size() != 1 || entities.at(second) != 20) {
    throw std::runtime_error("failed hash changed the existing entity map");
  }
}

int main()
{
  py::scoped_interpreter interpreter;
  try {
    py::exec(
      "import sys\n"
      "assert not sys._is_gil_enabled() and sys.flags.gil is None\n"
      "class Entity:\n"
      "    def __hash__(self):\n"
      "        return 7\n"
      "    def __eq__(self, other):\n"
      "        raise AssertionError('executor map called user equality')\n"
      "first, second = Entity(), Entity()\n");
    const auto globals = py::globals();
    const py::object first = globals["first"];
    const py::object second = globals["second"];
    check_distinct_entities(first, second);
    std::atomic<bool> failed{false};
    std::vector<std::thread> workers;
    for (size_t worker = 0; worker < 8; ++worker) {
      workers.emplace_back([&]() {
          py::gil_scoped_acquire attach;
          try {
            for (size_t round = 0; round < 256; ++round) {
              check_distinct_entities(first, second);
            }
          } catch (const std::exception & error) {
            std::cerr << error.what() << '\n';
            failed.store(true);
          }
        });
    }
    {
      py::gil_scoped_release release;
      for (auto & worker : workers) {
        worker.join();
      }
    }
    if (failed.load()) {
      throw std::runtime_error("native entity-map worker failed");
    }
    py::exec("assert not sys._is_gil_enabled() and sys.flags.gil is None");
    std::cout << "natural_GIL_OFF: 2048 independent map transactions on 8 threads; "
      "colliding hashes preserve distinct identities, user equality unused, hash errors preserved\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
