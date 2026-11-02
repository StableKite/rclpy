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

#ifndef RCLPY__EVENTS_EXECUTOR__PYTHON_HASHER_HPP_
#define RCLPY__EVENTS_EXECUTOR__PYTHON_HASHER_HPP_

#include <pybind11/pybind11.h>

namespace rclpy
{
namespace events_executor
{
/// This is intended to be used as the Hash template arg to STL containers using a
/// pybind11::handle as a Key.  This is the same hash that a native Python dict or set
/// would use given the same key.
struct PythonHasher
{
  inline auto operator()(const pybind11::handle & handle) const
  {
    return pybind11::hash(handle);
  }
};

/// Preserve the identity comparison historically used by pybind11::handle's
/// deprecated operator==. Python rich comparison could invoke user code and
/// would merge distinct executor entities that happen to compare equal.
struct PythonIdentityEqual
{
  bool operator()(const pybind11::handle & left, const pybind11::handle & right) const noexcept
  {
    return left.is(right);
  }
};
}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__PYTHON_HASHER_HPP_
