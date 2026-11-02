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

import gc
import threading
from concurrent.futures import ThreadPoolExecutor

import pytest
import rclpy.context as context_module
from rclpy import utilities
from rclpy.context import Context


def test_callback_reenters_context():
    before = context_module.g_logging_ref_count
    context = Context()
    context.init(args=[], domain_id=219)
    seen = []

    def callback():
        context.try_shutdown()
        assert context_module.g_logging_ref_count == before + 1
        seen.append(context.ok())

    context.on_shutdown(callback)
    try:
        context.shutdown()
        assert seen == [False]
        assert context_module.g_logging_ref_count == before
    finally:
        context.try_shutdown()
        context.destroy()


def test_callback_can_join_a_thread_using_context():
    context = Context()
    context.init(args=[], domain_id=219)
    seen = []
    worker = None

    def callback():
        nonlocal worker
        worker = threading.Thread(target=lambda: seen.append(context.ok()))
        worker.start()
        worker.join(2)
        assert not worker.is_alive(), 'callback retained the context state lock'

    context.on_shutdown(callback)
    try:
        context.shutdown()
        assert seen == [False]
    finally:
        context.try_shutdown()
        if worker is not None:
            worker.join(5)
        context.destroy()


def test_python_cleanup_after_native_shutdown():
    before = context_module.g_logging_ref_count
    context = Context()
    context.init(args=[], domain_id=219)
    seen = []
    context.on_shutdown(lambda: seen.append('closed'))
    try:
        with context.handle:
            context.handle.shutdown()
        assert not context.ok()
        context.try_shutdown()
        assert seen == ['closed']
        assert context_module.g_logging_ref_count == before
        context.try_shutdown()
        assert seen == ['closed']
    finally:
        context.destroy()


def test_callback_exception_still_releases_logging():
    before = context_module.g_logging_ref_count
    context = Context()
    context.init(args=[], domain_id=219)

    def callback():
        raise ValueError('expected user shutdown error')

    context.on_shutdown(callback)
    try:
        with pytest.raises(ValueError, match='expected user shutdown error'):
            context.shutdown()
        assert context_module.g_logging_ref_count == before
        context.try_shutdown()
    finally:
        context.destroy()


def test_concurrent_callback_registration_cleanup_and_collection():
    before = context_module.g_logging_ref_count
    context = Context()
    context.init(args=[], domain_id=219)
    seen = []
    barrier = threading.Barrier(9)

    class Owner:
        def __init__(self, value):
            self.value = value

        def callback(self):
            seen.append(self.value)

    owners = [Owner(index) for index in range(128)]
    for owner in owners:
        context.on_shutdown(owner.callback)
    del owner
    del owners[:64]
    gc.collect()

    def register(index):
        barrier.wait(timeout=10)
        for value in range(128):
            context.on_shutdown(lambda value=(index, value): seen.append(value))
            if value % 16 == 0:
                gc.collect()

    def shutdown():
        barrier.wait(timeout=10)
        context.try_shutdown()

    try:
        with ThreadPoolExecutor(max_workers=9) as pool:
            futures = [pool.submit(register, index) for index in range(8)]
            futures.append(pool.submit(shutdown))
            for future in futures:
                future.result(timeout=30)
        context.try_shutdown()
        assert sorted(value for value in seen if isinstance(value, int)) == list(
            range(64, 128))
        assert sorted(value for value in seen if isinstance(value, tuple)) == [
            (index, value) for index in range(8) for value in range(128)]
        assert context_module.g_logging_ref_count == before
    finally:
        context.try_shutdown()
        context.destroy()


def test_expired_callback_pruning_does_not_compare_callable_objects():
    context = Context()
    context.init(args=[], domain_id=219, initialize_logging=False)
    seen = []
    comparisons = []

    class CallableOwner:
        def __call__(self):
            seen.append('strong')
            return lambda: seen.append('unexpected return-value invocation')

        def __eq__(self, other):
            comparisons.append(other)
            context.ok()
            return False

    class WeakOwner:
        def callback(self):
            seen.append('expired')

    callback = CallableOwner()
    context.on_shutdown(callback)
    context.on_shutdown(callback)
    owner = WeakOwner()
    context.on_shutdown(owner.callback)
    try:
        del owner
        gc.collect()
        context.shutdown()
        assert seen == ['strong', 'strong']
        assert comparisons == []
    finally:
        context.try_shutdown()
        context.destroy()


def test_default_context_shutdown_callback_can_query_default_context():
    context = utilities.get_default_context()
    context.init(args=[], domain_id=219, initialize_logging=False)
    seen = []
    context.on_shutdown(lambda: seen.append(utilities.ok()))
    try:
        utilities.try_shutdown()
        assert seen == [False]
        assert utilities.get_default_context() is not context
    finally:
        utilities.try_shutdown()
        context.destroy()
