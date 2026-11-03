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
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "python_allocator.hpp"

namespace py = pybind11;

namespace
{
// Inspection mode only: wrap and forward the actual CPython allocator. The
// allocation is made on the attached main thread, respecting the old API's
// precondition. These contexts outlive the interpreter and all worker threads.
struct AllocatorObservation
{
  PyMemAllocatorEx original{};
  std::atomic<size_t> allocations{0};
  std::atomic<size_t> frees{0};
};
AllocatorObservation raw_observation;
AllocatorObservation mem_observation;
thread_local bool observing = false;

void * observed_malloc(void * context, size_t size)
{
  auto & state = *static_cast<AllocatorObservation *>(context);
  if (observing) {
    state.allocations.fetch_add(1);
  }
  return state.original.malloc(state.original.ctx, size);
}
void * observed_calloc(void * context, size_t count, size_t size)
{
  auto & state = *static_cast<AllocatorObservation *>(context);
  if (observing) {
    state.allocations.fetch_add(1);
  }
  return state.original.calloc(state.original.ctx, count, size);
}
void * observed_realloc(void * context, void * pointer, size_t size)
{
  auto & state = *static_cast<AllocatorObservation *>(context);
  if (observing) {
    state.allocations.fetch_add(1);
  }
  return state.original.realloc(state.original.ctx, pointer, size);
}
void observed_free(void * context, void * pointer)
{
  auto & state = *static_cast<AllocatorObservation *>(context);
  if (observing) {
    state.frees.fetch_add(1);
  }
  state.original.free(state.original.ctx, pointer);
}

class ForwardingAllocatorObservation
{
public:
  ForwardingAllocatorObservation()
  {
    PyMem_GetAllocator(PYMEM_DOMAIN_RAW, &raw_observation.original);
    PyMem_GetAllocator(PYMEM_DOMAIN_MEM, &mem_observation.original);
    PyMemAllocatorEx raw = {&raw_observation, observed_malloc, observed_calloc,
      observed_realloc, observed_free};
    PyMemAllocatorEx mem = {&mem_observation, observed_malloc, observed_calloc,
      observed_realloc, observed_free};
    PyMem_SetAllocator(PYMEM_DOMAIN_RAW, &raw);
    PyMem_SetAllocator(PYMEM_DOMAIN_MEM, &mem);
  }
  ~ForwardingAllocatorObservation()
  {
    PyMem_SetAllocator(PYMEM_DOMAIN_MEM, &mem_observation.original);
    PyMem_SetAllocator(PYMEM_DOMAIN_RAW, &raw_observation.original);
  }
};
}  // namespace

void inspect_allocation_domain()
{
  const bool attached = PyThreadState_GetUnchecked() != nullptr;
  {
    ForwardingAllocatorObservation hooks;
    observing = true;
    try {
      rclpy::PythonAllocator<uint64_t> allocator;
      auto pointer = allocator.allocate(32);
      allocator.deallocate(pointer, 32);
    } catch (...) {
      observing = false;
      throw;
    }
    observing = false;
  }
  std::cout << "{\"scope\":\"ATTACHED_MAIN_THREAD_DOMAIN_OBSERVATION_ONLY\","
    "\"thread_state_present\":" << (attached ? "true" : "false") <<
    ",\"mem_allocations\":" << mem_observation.allocations.load() <<
    ",\"mem_frees\":" << mem_observation.frees.load() <<
    ",\"raw_allocations\":" << raw_observation.allocations.load() <<
    ",\"raw_frees\":" << raw_observation.frees.load() << "}\n";
}

void check_element_count()
{
  rclpy::PythonAllocator<uint64_t> allocator;
  const size_t oversized = std::numeric_limits<size_t>::max() / sizeof(uint64_t) + 1;
  bool rejected = false;
  try {
    auto pointer = allocator.allocate(oversized);
    // Do not access storage returned for an invalid request. Return it through
    // the same actual allocator immediately before reporting the failure.
    allocator.deallocate(pointer, oversized);
  } catch (const std::bad_alloc &) {
    rejected = true;
  }
  if (!rejected) {
    throw std::runtime_error("native allocator wrapped an invalid element count instead of rejecting it");
  }
  auto zero = allocator.allocate(0);
  allocator.deallocate(zero, 0);
  allocator.deallocate(nullptr, 0);
  std::cout << "element count: oversized rejected, zero allocation/null free preserved\n";
}

void check_native_handoff()
{
  constexpr size_t worker_count = 8;
  constexpr size_t blocks = 1024;
  constexpr size_t words = 32;
  std::vector<std::vector<uint64_t *>> pointers(worker_count,
    std::vector<uint64_t *>(blocks, nullptr));
  std::mutex mutex;
  std::condition_variable cv;
  bool launch_finished = false;
  size_t active_workers = 0;
  size_t arrived = 0;
  std::atomic<bool> failed{false};
  std::vector<std::thread> threads;
  {
    py::gil_scoped_release detached;
    try {
      for (size_t worker = 0; worker < worker_count; ++worker) {
        threads.emplace_back([&, worker]() {
            if (PyThreadState_GetUnchecked() != nullptr) {
              failed.store(true);
            }
            rclpy::PythonAllocator<uint64_t> allocator;
            try {
              for (size_t block = 0; block < blocks; ++block) {
                pointers[worker][block] = allocator.allocate(words);
                for (size_t word = 0; word < words; ++word) {
                  pointers[worker][block][word] = worker * 1000000 + block * words + word;
                }
              }
            } catch (const std::exception & error) {
              std::cerr << "native allocation: " << error.what() << '\n';
              failed.store(true);
            }
            size_t neighbor;
            {
              std::unique_lock<std::mutex> lock(mutex);
              ++arrived;
              cv.notify_all();
              cv.wait(lock, [&]() {return launch_finished && arrived == active_workers;});
              neighbor = (worker + 1) % active_workers;
            }
            for (size_t block = 0; block < blocks; ++block) {
              auto pointer = pointers[neighbor][block];
              if (!pointer) {
                continue;
              }
              for (size_t word = 0; word < words; ++word) {
                if (pointer[word] != neighbor * 1000000 + block * words + word) {
                  failed.store(true);
                }
              }
              allocator.deallocate(pointer, words);
            }
            if (PyThreadState_GetUnchecked() != nullptr) {
              failed.store(true);
            }
          });
      }
    } catch (const std::exception & error) {
      std::cerr << "native worker launch: " << error.what() << '\n';
      failed.store(true);
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      active_workers = threads.size();
      launch_finished = true;
      cv.notify_all();
    }
    for (auto & thread : threads) {
      thread.join();
    }
  }
  if (failed.load() || active_workers != worker_count) {
    throw std::runtime_error("actual native buffer allocation/cross-thread release failed");
  }
  std::cout << "native handoff: 8 unattached threads/8192 buffers/262144 payload words; "
    "every buffer released by a different thread; no thread state attached\n";
}

int main(int argc, char ** argv)
{
  auto release_native = [](uint64_t * pointer) {
      rclpy::PythonAllocator<uint64_t>().deallocate(pointer, 32);
    };
  std::unique_ptr<uint64_t, decltype(release_native)> late_owner(nullptr, release_native);
  {
    py::scoped_interpreter interpreter;
    try {
      py::exec("import sys\nassert not sys._is_gil_enabled() and sys.flags.gil is None");
      if (argc == 2 && std::string(argv[1]) == "--inspect-domain") {
        inspect_allocation_domain();
      } else if (argc == 2 && std::string(argv[1]) == "--element-count") {
        check_element_count();
      } else if (argc == 1) {
        check_element_count();
        check_native_handoff();
        late_owner.reset(rclpy::PythonAllocator<uint64_t>().allocate(32));
        for (size_t word = 0; word < 32; ++word) {
          late_owner.get()[word] = 9000000 + word;
        }
      } else {
        throw std::runtime_error("unknown native allocator test mode");
      }
      py::exec("assert not sys._is_gil_enabled() and sys.flags.gil is None");
    } catch (const std::exception & error) {
      // Python exceptions must be inspected before the interpreter finalizes.
      std::cerr << error.what() << '\n';
      return 1;
    }
  }
  if (late_owner) {
    std::atomic<bool> failed{false};
    try {
      std::thread worker([owner = std::move(late_owner), &failed]() mutable {
          if (Py_IsInitialized() || PyThreadState_GetUnchecked() != nullptr) {
            failed.store(true);
          }
          for (size_t word = 0; word < 32; ++word) {
            if (owner.get()[word] != 9000000 + word) {
              failed.store(true);
            }
          }
          owner.reset();
          if (PyThreadState_GetUnchecked() != nullptr) {
            failed.store(true);
          }
        });
      worker.join();
    } catch (const std::exception & error) {
      std::cerr << "late native release: " << error.what() << '\n';
      return 1;
    }
    if (failed.load()) {
      std::cerr << "native buffer could not outlive the initialized interpreter\n";
      return 1;
    }
    std::cout << "late release: actual buffer payload retained and freed on an unattached "
      "native worker after interpreter finalization\n";
  }
  return 0;
}
