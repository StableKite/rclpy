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

from concurrent.futures import ThreadPoolExecutor
import threading

from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as native
from rclpy.signals import SignalHandlerGuardCondition


def test_concurrent_signal_guard_registration_trigger_and_cleanup():
    context = Context()
    context.init(args=[], domain_id=219)
    barrier = threading.Barrier(8)

    def worker(_):
        barrier.wait(timeout=10)
        for _ in range(64):
            guard = SignalHandlerGuardCondition(context)
            guard.trigger()
            guard.destroy()
            guard.destroy()
        return 64

    try:
        with ThreadPoolExecutor(max_workers=8) as executor:
            assert sum(executor.map(worker, range(8))) == 512
    finally:
        context.shutdown()


def test_native_guard_keeps_context_alive_after_owner_close():
    context = native.Context([], 219)
    guard = native.GuardCondition(context)
    try:
        context.shutdown()
        context.destroy_when_not_in_use()
        assert not context.ok()
        guard.trigger_guard_condition()
    finally:
        guard.destroy_when_not_in_use()
