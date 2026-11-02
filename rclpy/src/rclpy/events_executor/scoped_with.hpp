// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
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

#ifndef RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
#define RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_

#include <pybind11/pybind11.h>

#include <exception>
#include <utility>

namespace rclpy
{
namespace events_executor
{

/// Enters a python context manager for the scope of this object instance.
class ScopedWith
{
public:
  explicit ScopedWith(pybind11::handle object)
  {
    pybind11::gil_scoped_acquire attached;
    auto owner = pybind11::reinterpret_borrow<pybind11::object>(object);
    owner.attr("__enter__")();
    object_ = std::move(owner);
  }

  ScopedWith(const ScopedWith &) = delete;
  ScopedWith & operator=(const ScopedWith &) = delete;

  ~ScopedWith() noexcept
  {
#if PY_VERSION_HEX >= 0x030D0000
    const bool thread_attached = PyThreadState_GetUnchecked() != nullptr;
    const bool finalizing = Py_IsFinalizing() != 0;
#else
    const bool thread_attached = PyGILState_Check() != 0;
    const bool finalizing = _Py_IsFinalizing() != 0;
#endif
    if (!thread_attached && (!Py_IsInitialized() || finalizing)) {
      // A foreign thread cannot reattach during interpreter shutdown. As with
      // other finalization-time Python owners, leave this reference to process
      // teardown instead of invoking Python after its runtime has stopped.
      (void)object_.release();
      return;
    }
    pybind11::gil_scoped_acquire attached;
    PyObject * type = nullptr;
    PyObject * value = nullptr;
    PyObject * traceback = nullptr;
    PyErr_Fetch(&type, &value, &traceback);
    try {
      object_.attr("__exit__")(pybind11::none(), pybind11::none(), pybind11::none());
    } catch (pybind11::error_already_set & error) {
      error.discard_as_unraisable(object_);
    } catch (const std::exception & error) {
      PyErr_SetString(PyExc_RuntimeError, error.what());
      PyErr_WriteUnraisable(object_.ptr());
    }
    // DECREF can run Python cleanup too, so clear the member while attached.
    object_ = pybind11::object();
    PyErr_Restore(type, value, traceback);
  }

private:
  pybind11::object object_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
