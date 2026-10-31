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

#include <pybind11/pybind11.h>

#include <atomic>
#include <barrier>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "destroyable.hpp"
#include "exceptions.hpp"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class Probe : public rclpy::Destroyable
{
public:
  explicit Probe(std::atomic<size_t> & cleanups)
  : cleanups_(&cleanups) {}

  void destroy() override
  {
    cleanups_->fetch_add(1);
    // Cleanup is deliberately reentrant. Holding the lifecycle mutex while
    // invoking this virtual function would deadlock, even on a single thread.
    destroy_when_not_in_use();
  }

private:
  std::atomic<size_t> * cleanups_;
};
}  // namespace

int main()
{
  constexpr size_t rounds = 128;
  constexpr size_t workers = 8;
  for (size_t round = 0; round < rounds; ++round) {
    std::atomic<size_t> cleanups{0};
    Probe probe(cleanups);
    probe.enter();
    std::barrier entered(static_cast<std::ptrdiff_t>(workers + 1));
    std::barrier requested(static_cast<std::ptrdiff_t>(workers + 1));
    std::vector<std::thread> threads;
    for (size_t index = 0; index < workers; ++index) {
      threads.emplace_back([&]() {
          probe.enter();
          entered.arrive_and_wait();
          for (size_t request = 0; request < 16; ++request) {
            probe.destroy_when_not_in_use();
          }
          requested.arrive_and_wait();
          probe.exit({}, {}, {});
        });
    }
    entered.arrive_and_wait();
    requested.arrive_and_wait();
    require(cleanups.load() == 0, "cleanup ran while the main lease was still held");
    probe.exit({}, {}, {});
    for (auto & thread : threads) {
      thread.join();
    }
    require(cleanups.load() == 1, "concurrent destruction must run cleanup exactly once");
    probe.destroy_when_not_in_use();
    require(cleanups.load() == 1, "repeated destruction ran cleanup again");
    bool rejected = false;
    try {
      probe.enter();
    } catch (const rclpy::InvalidHandle &) {
      rejected = true;
    }
    require(rejected, "enter after a destruction request must reject the lease");

    // Copy construction creates an independent lifecycle, including when the
    // source is already destroyed. This is required by native rclpy entities.
    Probe copy(probe);
    copy.enter();
    copy.destroy_when_not_in_use();
    require(cleanups.load() == 1, "the copied object's active lease was ignored");
    copy.exit({}, {}, {});
    require(cleanups.load() == 2, "the copied object's cleanup did not run once");
    bool unbalanced = false;
    try {
      copy.exit({}, {}, {});
    } catch (const std::runtime_error &) {
      unbalanced = true;
    }
    require(unbalanced, "unbalanced exit must report an error");
  }
  std::cout << "Native Destroyable: " << rounds << " rounds, " << workers
            << " OS threads, 16384 concurrent cleanup requests; all invariants passed.\n";
  return 0;
}
