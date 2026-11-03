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

#ifndef RCLPY__CLEANUP_WARNING_HPP_
#define RCLPY__CLEANUP_WARNING_HPP_

#include <pybind11/pybind11.h>

#include <cstdio>
#include <optional>

namespace rclpy
{

/// Report SDK cleanup failure without replacing an active Python exception.
/// The caller must release native registry/resource locks before reporting.
inline void warn_cleanup_error(
  const char * operation, const char * resource, const char * detail) noexcept
{
  const bool initialized = Py_IsInitialized() != 0;
#if PY_VERSION_HEX >= 0x030D0000
  const bool attached = initialized && PyThreadState_GetUnchecked() != nullptr;
  const bool finalizing = Py_IsFinalizing() != 0;
#else
  const bool attached = initialized && PyGILState_Check() != 0;
  const bool finalizing = _Py_IsFinalizing() != 0;
#endif
  if (!initialized || (!attached && finalizing)) {
    std::fprintf(stderr, "rclpy failed to %s %s: %s\n", operation, resource, detail);
    return;
  }
  try {
    std::optional<pybind11::gil_scoped_acquire> attachment;
    if (!attached) {
      attachment.emplace();
    }
    PyObject * type = nullptr;
    PyObject * value = nullptr;
    PyObject * traceback = nullptr;
    PyErr_Fetch(&type, &value, &traceback);
    if (PyErr_WarnFormat(
        PyExc_RuntimeWarning, 1, "Failed to %s %s: %s", operation, resource, detail) < 0)
    {
      // A destructor cannot propagate a warnings-as-errors exception. Report it
      // visibly through the normal unraisable hook, then restore the caller.
      PyErr_WriteUnraisable(Py_None);
    }
    PyErr_Restore(type, value, traceback);
  } catch (...) {
    std::fprintf(stderr, "rclpy failed to %s %s: %s\n", operation, resource, detail);
  }
}

}  // namespace rclpy

#endif  // RCLPY__CLEANUP_WARNING_HPP_
