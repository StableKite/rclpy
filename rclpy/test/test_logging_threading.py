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

import threading
import warnings
from concurrent.futures import ThreadPoolExecutor

import pytest
from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as native


def test_invalid_native_node_constructor_has_no_cleanup_warning():
    context = Context()
    context.init(args=[], domain_id=219)
    try:
        with warnings.catch_warnings(record=True) as recorded:
            warnings.simplefilter('always')
            for name, namespace in [('bad node', '/'), ('0bad', '/'),
                                    ('bad/name', '/'), ('valid', 'bad namespace')]:
                with context.handle:
                    with pytest.raises(ValueError):
                        native.Node(name, namespace, context.handle, [], False, True)
        assert not recorded, [str(warning.message) for warning in recorded]
    finally:
        context.try_shutdown()
        context.destroy()


def test_concurrent_native_logging_levels_and_messages():
    context = Context()
    context.init(args=[], domain_id=219)
    barrier = threading.Barrier(8)

    def logger(index):
        barrier.wait(timeout=10)
        for value in range(32):
            name = 'sk.threaded_logger.' + str(index) + '.' + str(value)
            native.rclpy_logging_set_logger_level(name, 20)
            assert native.rclpy_logging_get_logger_level(name) == 20
            assert native.rclpy_logging_get_logger_effective_level(
                name + '.child') == 20
            assert native.rclpy_logging_logger_is_enabled_for(name, 20)
            assert not native.rclpy_logging_logger_is_enabled_for(name, 10)
            native.rclpy_logging_rcutils_log(
                20, name, 'native message', 'logger', __file__, 1)
        return 32

    try:
        with ThreadPoolExecutor(max_workers=8) as pool:
            assert sum(pool.map(logger, range(8))) == 256
    finally:
        context.try_shutdown()
        context.destroy()
