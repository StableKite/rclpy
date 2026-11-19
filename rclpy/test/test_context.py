# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import subprocess
import sys

import pytest
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node


def test_on_shutdown_method() -> None:
    context = Context()
    context.init()
    assert context.ok()

    callback_called = False

    class SomeClass:

        def on_shutdown(self) -> None:
            nonlocal callback_called
            callback_called = True

    instance = SomeClass()
    context.on_shutdown(instance.on_shutdown)

    context.shutdown()
    assert not context.ok()

    assert callback_called


def test_on_shutdown_function() -> None:
    context = Context()
    context.init()
    assert context.ok()

    callback_called = False

    def on_shutdown() -> None:
        nonlocal callback_called
        callback_called = True

    context.on_shutdown(on_shutdown)

    context.shutdown()
    assert not context.ok()

    assert callback_called


def test_context_manager() -> None:
    context = Context()

    assert not context.ok(), 'the context should not be ok() before init() is called'

    context.init()

    with context as the_context:
        # Make sure the correct instance is returned
        assert the_context is context

        assert context.ok(), 'the context should now be initialized'

    assert not context.ok(), 'the context should now be shut down'

    # Make sure it does not raise (smoke test)
    context.try_shutdown()


def test_shutdown_destroys_all_tracked_nodes() -> None:
    context = Context()
    context.init()

    nodes = [Node(f'test_node_{i}', context=context) for i in range(4)]
    assert all(node.context is context for node in nodes)

    context.shutdown()

    # Every node tracked by the context must be destroyed on shutdown
    for node in nodes:
        with pytest.raises(InvalidHandle):
            # handle invalid because it was destroyed by Context.shutdown()
            with node.handle:
                pass


@pytest.mark.skipif(sys.platform == 'win32', reason='requires POSIX signal delivery')
def test_signal_shutdown_after_context_destruction() -> None:
    subprocess.run([sys.executable, '-c', '''
import os
import signal
import time

from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions, install_signal_handlers, uninstall_signal_handlers

signal.signal(signal.SIGINT, lambda *_: None)
install_signal_handlers(SignalHandlerOptions.SIGINT)
try:
    retired = Context()
    retired.init(args=[], initialize_logging=False)
    retired.destroy()

    live = Context()
    live.init(args=[], initialize_logging=False)
    os.kill(os.getpid(), signal.SIGINT)
    deadline = time.monotonic() + 5
    while live.ok() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not live.ok()
    live.destroy()
finally:
    uninstall_signal_handlers()
'''], check=True, timeout=15)
