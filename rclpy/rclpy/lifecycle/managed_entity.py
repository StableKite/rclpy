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

from typing import Callable, Concatenate, Optional, ParamSpec, TYPE_CHECKING, TypeVar, overload

import rclpy.lifecycle
if TYPE_CHECKING:
    import rclpy.lifecycle.node
from functools import wraps

from ..impl.implementation_singleton import rclpy_implementation as _rclpy


TransitionCallbackReturn = _rclpy.TransitionCallbackReturnType


_Params = ParamSpec('_Params')
_EntityT = TypeVar('_EntityT', bound='SimpleManagedEntity')


class ManagedEntity:

    def on_configure(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle configure transition request."""
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle cleanup transition request."""
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle shutdown transition request."""
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle activate transition request."""
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle deactivate transition request."""
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        """Handle error transition request."""
        return TransitionCallbackReturn.SUCCESS


class SimpleManagedEntity(ManagedEntity):
    """A simple managed entity that only sets a flag when activated/deactivated."""

    def __init__(self) -> None:
        self._enabled = False

    def on_activate(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        self._enabled = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.node.LifecycleState) -> TransitionCallbackReturn:
        self._enabled = False
        return TransitionCallbackReturn.SUCCESS

    @property
    def is_activated(self) -> bool:
        return self._enabled

    @staticmethod
    @overload
    def when_enabled(
        wrapped: Callable[Concatenate[_EntityT, _Params], object], *,
        when_not_enabled: Optional[Callable[[], object]] = None
    ) -> Callable[Concatenate[_EntityT, _Params], None]: ...

    @staticmethod
    @overload
    def when_enabled(
        wrapped: None = None, *, when_not_enabled: Optional[Callable[[], object]] = None
    ) -> Callable[[Callable[Concatenate[_EntityT, _Params], object]], Callable[Concatenate[_EntityT, _Params], None]]: ...

    @staticmethod
    def when_enabled(
        wrapped: Optional[Callable[Concatenate[_EntityT, _Params], object]] = None, *,
        when_not_enabled: Optional[Callable[[], object]] = None
    ) -> Callable[Concatenate[_EntityT, _Params], None] | Callable[[Callable[Concatenate[_EntityT, _Params], object]], Callable[Concatenate[_EntityT, _Params], None]]:
        def decorator(
            wrapped: Callable[Concatenate[_EntityT, _Params], object]
        ) -> Callable[Concatenate[_EntityT, _Params], None]:
            @wraps(wrapped)
            def only_when_enabled_wrapper(self: _EntityT, *args: _Params.args, **kwargs: _Params.kwargs) -> None:
                if not self._enabled:
                    if when_not_enabled is not None:
                        when_not_enabled()
                    return
                wrapped(self, *args, **kwargs)
            return only_when_enabled_wrapper
        if wrapped is None:
            return decorator
        return decorator(wrapped)
