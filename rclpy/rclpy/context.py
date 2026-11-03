# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from inspect import ismethod
import sys
import threading
from types import MethodType, TracebackType
from typing import Callable
from typing import ContextManager
from typing import Dict
from typing import List
from typing import Optional
from typing import Protocol
from typing import Type
from typing import Union
from weakref import WeakMethod

from rclpy import native_types
from rclpy.destroyable import DestroyableType

_ShutdownCallback = Callable[[], object]


class _StrongCallback:
    __slots__ = ('callback',)

    def __init__(self, callback: _ShutdownCallback) -> None:
        self.callback = callback


_CallbackReference = Union['WeakMethod[MethodType]', _StrongCallback]


class ContextHandle(DestroyableType, Protocol):

    def ok(self) -> bool:
        ...

    def get_domain_id(self) -> int:
        ...

    def shutdown(self) -> None:
        ...


g_logging_configure_lock = threading.Lock()
g_logging_ref_count: int = 0


class Context(ContextManager['Context']):
    """
    Encapsulates the lifecycle of init and shutdown.

    Context objects should not be reused, and are finalized in their destructor.
    Wraps the `rcl_context_t` type.

    :Example:
        >>> from rclpy.context import Context
        >>> with Context() as context:
        >>>     context.ok()
        True

    """

    def __init__(self) -> None:
        # Weak-reference hooks can re-enter this thread during collection.
        self._lock = threading.RLock()
        # Serialize complete shutdown transactions while letting callbacks use
        # the context state lock (including from another thread).
        self._shutdown_lock = threading.RLock()
        self._cleanup_active = False
        self._callbacks: Dict[object, _CallbackReference] = {}
        self._logging_initialized = False
        self.__context: Optional[native_types.Context] = None

    @property
    def handle(self) -> Optional[native_types.Context]:
        return self.__context

    def destroy(self) -> None:
        with self._shutdown_lock:
            with self._lock:
                context = self.__context
            if context:
                context.destroy_when_not_in_use()

    def init(self,
             args: Optional[List[str]] = None,
             *,
             initialize_logging: bool = True,
             domain_id: Optional[int] = None) -> None:
        """
        Initialize ROS communications for a given context.

        :param args: List of command line arguments.
        """
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

        global g_logging_ref_count
        with self._lock:
            if domain_id is not None and domain_id < 0:
                raise RuntimeError(
                    'Domain id ({}) should not be lower than zero.'
                    .format(domain_id))

            if self.__context is not None:
                raise RuntimeError('Context.init() must only be called once')

            self.__context = _rclpy.Context(
                args if args is not None else sys.argv,
                domain_id if domain_id is not None else _rclpy.RCL_DEFAULT_DOMAIN_ID)
            if initialize_logging and not self._logging_initialized:
                with g_logging_configure_lock:
                    g_logging_ref_count += 1
                    if g_logging_ref_count == 1:
                        _rclpy.rclpy_logging_configure(self.__context)
                self._logging_initialized = True

    def ok(self) -> bool:
        """Check if context hasn't been shut down."""
        with self._lock:
            if self.__context is None:
                return False
            with self.__context:
                return self.__context.ok()

    def _call_on_shutdown_callbacks(self) -> None:
        replacement: Dict[object, _CallbackReference] = {}
        with self._lock:
            callbacks = self._callbacks
            self._callbacks = replacement
        for entry in callbacks.values():
            callback = entry.callback if isinstance(entry, _StrongCallback) else entry()
            if callback is not None:
                callback()

    def shutdown(self) -> None:
        """Shutdown this context."""
        self._shutdown(try_only=False)

    def try_shutdown(self) -> None:
        """Shutdown this context, if not already shutdown."""
        self._shutdown(try_only=True)

    def _shutdown(self, *, try_only: bool) -> None:
        with self._shutdown_lock:
            with self._lock:
                context = self.__context
                if context is None:
                    if try_only:
                        return
                    raise RuntimeError('Context must be initialized before it can be shutdown')
                if try_only and self._cleanup_active:
                    # A callback may request idempotent shutdown recursively.
                    return
            with context:
                with self._lock:
                    if not try_only or context.ok():
                        context.shutdown()
                    self._cleanup_active = True
                try:
                    # Also required when the native signal worker already
                    # stopped RCL. User code never runs under the state lock.
                    self._call_on_shutdown_callbacks()
                finally:
                    with self._lock:
                        try:
                            self._logging_fini()
                        finally:
                            self._cleanup_active = False

    def _remove_callback(self, weak_method: 'WeakMethod[MethodType]') -> None:
        with self._lock:
            # Never compare user callables while pruning a weak owner. The
            # detached shutdown snapshot may already have consumed this key.
            self._callbacks.pop(id(weak_method), None)

    def on_shutdown(self, callback: _ShutdownCallback) -> None:
        """Add a callback to be called on shutdown; its return value is ignored."""
        if not callable(callback):
            raise TypeError('callback should be a callable, got {}', type(callback))

        # Allocate weak references outside the state lock: their construction
        # can trigger collection. Opaque keys never hash/compare user callables
        # and retain duplicate registrations in insertion order.
        key: object
        entry: _CallbackReference
        if ismethod(callback):
            weak_method = WeakMethod(callback, self._remove_callback)
            key = id(weak_method)
            entry = weak_method
        else:
            entry = _StrongCallback(callback)
            key = entry
        call_now = False
        with self._lock:
            context = self.__context
            if context is None:
                self._callbacks[key] = entry
            else:
                with context:
                    if not context.ok():
                        call_now = True
                    else:
                        self._callbacks[key] = entry
        if call_now:
            callback()

    def _logging_fini(self) -> None:
        # This function must be called with self._lock held.
        from rclpy.impl.implementation_singleton import rclpy_implementation
        global g_logging_ref_count
        if self._logging_initialized:
            with g_logging_configure_lock:
                g_logging_ref_count -= 1
                if g_logging_ref_count == 0:
                    rclpy_implementation.rclpy_logging_fini()
                if g_logging_ref_count < 0:
                    raise RuntimeError(
                        'Unexpected error: logger ref count should never be lower that zero')
            self._logging_initialized = False

    def get_domain_id(self) -> int:
        """Get domain id of context."""
        if self.__context is None:
            raise RuntimeError('Context must be initialized before it can have a domain id')
        with self.__context, self._lock:
            return self.__context.get_domain_id()

    def __enter__(self) -> 'Context':
        # We do not accept parameters here. If one wants to customize the init() call,
        # they would have to call it manually and not use the ContextManager convenience
        self.init()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.try_shutdown()
