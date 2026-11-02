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

import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from rclpy.context import Context
from rclpy.signals import (
    SignalHandlerGuardCondition,
    SignalHandlerOptions,
    get_current_signal_handlers_options,
    install_signal_handlers,
    uninstall_signal_handlers,
)


def test_concurrent_signal_lifecycle():
    previous = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda *_: None)
    barrier = threading.Barrier(8)

    def toggle(index):
        barrier.wait(timeout=10)
        for _ in range(32):
            install_signal_handlers(
                SignalHandlerOptions.SIGTERM if index % 2 else SignalHandlerOptions.NO)
            assert int(get_current_signal_handlers_options()) in (0, 1, 2, 3)
            uninstall_signal_handlers()
        return 32

    try:
        with ThreadPoolExecutor(max_workers=8) as pool:
            assert sum(pool.map(toggle, range(8))) == 256
        assert get_current_signal_handlers_options() == SignalHandlerOptions.NO
    finally:
        uninstall_signal_handlers()
        signal.signal(signal.SIGTERM, previous)


def test_repeated_signal_chaining_and_native_context_shutdown():
    previous = signal.getsignal(signal.SIGTERM)
    seen = []
    signal.signal(signal.SIGTERM, lambda number, frame: seen.append(number))
    context = Context()
    context.init(args=[])
    guard = SignalHandlerGuardCondition(context=context)
    try:
        install_signal_handlers(SignalHandlerOptions.SIGTERM)
        for _ in range(32):
            signal.raise_signal(signal.SIGTERM)
        deadline = time.monotonic() + 10
        while context.ok() and time.monotonic() < deadline:
            time.sleep(0.001)
        assert not context.ok()
        assert seen == [signal.SIGTERM] * 32
        uninstall_signal_handlers()
        signal.raise_signal(signal.SIGTERM)
        assert seen == [signal.SIGTERM] * 33
    finally:
        uninstall_signal_handlers()
        guard.destroy()
        context.try_shutdown()
        signal.signal(signal.SIGTERM, previous)
