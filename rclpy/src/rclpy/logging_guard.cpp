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

#include "logging.hpp"

namespace rclpy
{
std::recursive_mutex LoggingGuard::logging_mutex_;

LoggingGuard::LoggingGuard()
: guard_(logging_mutex_, std::defer_lock)
{
#if PY_VERSION_HEX >= 0x030D0000
  if (Py_IsInitialized() && PyThreadState_GetUnchecked() != nullptr) {
#else
  if (Py_IsInitialized() && PyGILState_Check()) {
#endif
    release_.emplace();
  }
  guard_.lock();
}
}  // namespace rclpy
