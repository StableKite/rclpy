# Copyright 2026 StableKite contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as native
from rclpy.signals import SignalHandlerGuardCondition


@pytest.fixture
def context():
    context = Context()
    context.init(args=[], domain_id=219)
    try:
        yield context
    finally:
        context.shutdown()


def test_signal_guard_destroy_is_idempotent(context):
    guard = SignalHandlerGuardCondition(context)
    guard.destroy()
    guard.destroy()
    with pytest.raises(InvalidHandle):
        guard.trigger()


def test_already_unregistered_signal_guard_still_releases_native_handle(context):
    guard = SignalHandlerGuardCondition(context)
    with guard.handle:
        native.unregister_sigint_guard_condition(guard.handle)
    guard.destroy()
    with pytest.raises(InvalidHandle):
        guard.trigger()


def test_executor_finalization_after_signal_guard_cleanup(context):
    executor = SingleThreadedExecutor(context=context)
    try:
        executor._sigint_gc.destroy()
        executor.__del__()
    finally:
        executor.shutdown()


def test_executor_finalizer_before_signal_guard_initialization():
    # A failed __init__ still causes Python to finalize the allocated instance.
    executor = SingleThreadedExecutor.__new__(SingleThreadedExecutor)
    executor.__del__()
