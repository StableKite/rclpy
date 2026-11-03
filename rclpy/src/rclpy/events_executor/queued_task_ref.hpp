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

#ifndef RCLPY__EVENTS_EXECUTOR__QUEUED_TASK_REF_HPP_
#define RCLPY__EVENTS_EXECUTOR__QUEUED_TASK_REF_HPP_

#include <pybind11/pybind11.h>

namespace rclpy
{
namespace events_executor
{

/// A queue entry observes a task; the executor's GC-visible dictionary owns it.
class QueuedTaskRef
{
public:
  explicit QueuedTaskRef(pybind11::handle task)
  {
    pybind11::gil_scoped_acquire attached;
    reference_ = pybind11::weakref(task);
  }

  QueuedTaskRef(const QueuedTaskRef &) = delete;
  QueuedTaskRef & operator=(const QueuedTaskRef &) = delete;

  ~QueuedTaskRef() noexcept
  {
#if PY_VERSION_HEX >= 0x030D0000
    const bool thread_attached = PyThreadState_GetUnchecked() != nullptr;
    const bool finalizing = Py_IsFinalizing() != 0;
#else
    const bool thread_attached = PyGILState_Check() != 0;
    const bool finalizing = _Py_IsFinalizing() != 0;
#endif
    if (!thread_attached && (!Py_IsInitialized() || finalizing)) {
      // A foreign thread cannot reattach after interpreter finalization starts.
      // No task is owned here; leave only this weak-reference handle to teardown.
      (void)reference_.release();
      return;
    }
    pybind11::gil_scoped_acquire attached;
    reference_ = pybind11::weakref();
  }

  /// The caller must be attached while resolving and using the returned owner.
  pybind11::object lock() const {return reference_();}

private:
  pybind11::weakref reference_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__QUEUED_TASK_REF_HPP_
