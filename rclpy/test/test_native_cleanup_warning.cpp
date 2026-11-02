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

#include <algorithm>
#include <atomic>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "context.hpp"

namespace py = pybind11;

void require(bool value, const char * message)
{
  if (!value) {
    throw std::runtime_error(message);
  }
}

struct PreparedOptions
{
  std::unique_ptr<rclpy::InitOptions> owner;
  rcl_init_options_t retained;

  PreparedOptions()
  : owner(std::make_unique<rclpy::InitOptions>(rcl_get_default_allocator())),
    retained(owner->rcl_options)
  {
    owner->rcl_options = rcl_get_zero_initialized_init_options();
  }

  PreparedOptions(const PreparedOptions &) = delete;
  PreparedOptions & operator=(const PreparedOptions &) = delete;

  void release()
  {
    const auto result = rcl_init_options_fini(&retained);
    require(result == RCL_RET_OK, "retained real SDK resource did not finalize");
    retained = rcl_get_zero_initialized_init_options();
  }

  ~PreparedOptions()
  {
    owner.reset();
    if (retained.impl != nullptr) {
      const auto result = rcl_init_options_fini(&retained);
      if (result != RCL_RET_OK) {
        std::cerr << "fallback retained SDK allocation cleanup failed\n";
      }
    }
  }
};

int after_finalization()
{
  std::unique_ptr<PreparedOptions> item;
  {
    py::scoped_interpreter interpreter;
    require(!py::cast<bool>(py::module_::import("sys").attr("_is_gil_enabled")()),
      "natural GIL OFF required");
    item = std::make_unique<PreparedOptions>();
  }
  require(!Py_IsInitialized(), "interpreter has not finalized");
  std::thread worker([owner = std::move(item->owner)]() mutable {owner.reset();});
  worker.join();
  require(!Py_IsInitialized(), "native cleanup initialized Python");
  item->release();
  std::cout << "PASS post-finalization actual SDK error/native diagnostic/no Python reattach\n";
  return 0;
}

int active(const std::string & mode)
{
  py::scoped_interpreter interpreter;
  try {
    auto sys = py::module_::import("sys");
    require(!py::cast<bool>(sys.attr("_is_gil_enabled")()), "natural GIL OFF required");
    require(sys.attr("flags").attr("gil").is_none(), "GIL override found");
    py::dict globals;
    globals["sys"] = sys;
    py::exec(
      "import warnings\n"
      "warnings.simplefilter('error', RuntimeWarning)\n"
      "original_unraisable = sys.unraisablehook\n"
      "observed = []\n"
      "def observe_unraisable(args):\n"
      "    observed.append(args.exc_type.__name__)\n"
      "    original_unraisable(args)\n"
      "sys.unraisablehook = observe_unraisable\n", globals);
    const size_t count = mode == "foreign" ? 8 : 1;
    std::vector<std::unique_ptr<PreparedOptions>> resources;
    for (size_t i = 0; i < count; ++i) {
      resources.push_back(std::make_unique<PreparedOptions>());
    }
    py::object sentinel = py::module_::import("builtins").attr("ValueError")("original exception identity");
    py::object capture;
    py::object recorded;
    if (mode == "normal") {
      auto warnings = py::module_::import("warnings");
      capture = warnings.attr("catch_warnings")(py::arg("record") = true);
      recorded = capture.attr("__enter__")();
      warnings.attr("simplefilter")(
        "always", py::reinterpret_borrow<py::object>(PyExc_RuntimeWarning));
    }
    if (mode == "preserve") {
      PyErr_SetObject(PyExc_ValueError, sentinel.ptr());
    }
    std::atomic<size_t> unattached{0};
    if (mode == "foreign") {
      std::vector<std::thread> workers;
      {
        py::gil_scoped_release released;
        for (auto & resource : resources) {
          workers.emplace_back([owner = std::move(resource->owner), &unattached]() mutable {
            if (PyThreadState_GetUnchecked() == nullptr) {
              unattached.fetch_add(1);
            }
            owner.reset();
            if (PyThreadState_GetUnchecked() == nullptr) {
              unattached.fetch_add(1);
            }
          });
        }
        for (auto & worker : workers) {
          worker.join();
        }
      }
    } else {
      resources[0]->owner.reset();
    }
    PyObject * type = nullptr;
    PyObject * value = nullptr;
    PyObject * traceback = nullptr;
    PyErr_Fetch(&type, &value, &traceback);
    const bool correct_exception = mode == "preserve" ?
      type == PyExc_ValueError && value == sentinel.ptr() : type == nullptr;
    Py_XDECREF(type);
    Py_XDECREF(value);
    Py_XDECREF(traceback);
    for (auto & resource : resources) {
      resource->release();
    }
    if (mode == "normal") {
      capture.attr("__exit__")(py::none(), py::none(), py::none());
      require(py::len(recorded) == 1, "ordinary warning was not observable");
      require(recorded[py::int_(0)].attr("category").ptr() == PyExc_RuntimeWarning,
        "ordinary warning category changed");
    }
    sys.attr("unraisablehook") = globals["original_unraisable"];
    require(correct_exception, "cleanup replaced or left a pending exception");
    const py::ssize_t reports = py::len(globals["observed"]);
    require(reports == static_cast<py::ssize_t>(mode == "normal" ? 0 : count), "unraisable reporting count changed");
    for (py::handle name : py::cast<py::list>(globals["observed"])) {
      require(py::cast<std::string>(name) == "RuntimeWarning", "reported error category changed");
    }
    if (mode == "foreign") {
      require(unattached == 2 * count, "foreign cleanup retained Python thread state");
    }
    require(!py::cast<bool>(sys.attr("_is_gil_enabled")()), "cleanup enabled the GIL");
    std::cout << "PASS mode=" << mode << " actual_SDK_errors=" << count
              << " exception_identity_preserved=true unraisable_reports=" << reports
              << " SDK_allocations_finalized_once=true natural_GIL_OFF=true\n";
    return 0;
  } catch (py::error_already_set & error) {
    error.discard_as_unraisable("cleanup warning probe");
    return 1;
  } catch (const std::exception & error) {
    if (PyErr_Occurred() != nullptr) {
      PyErr_Print();
    }
    std::cerr << error.what() << '\n';
    return 1;
  }
}

int main(int argc, char ** argv)
{
  const std::string mode = argc == 2 ? argv[1] : "error";
  if (mode == "finalized") {
    try {return after_finalization();} catch (const std::exception & error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
  }
  require(mode == "error" || mode == "normal" || mode == "preserve" || mode == "foreign",
    "unknown probe mode");
  return active(mode);
}
