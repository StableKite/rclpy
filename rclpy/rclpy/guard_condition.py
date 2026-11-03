# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from typing import Callable, cast

from rclpy.callback_groups import CallbackGroup
from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.utilities import get_default_context


class GuardCondition:

    def __init__(
        self, callback: Callable[[], object] | None,
        callback_group: CallbackGroup | None, context: Context | None = None
    ) -> None:
        self._context = get_default_context() if context is None else context
        with cast(_rclpy.Context, self._context.handle):
            self.__gc = _rclpy.GuardCondition(cast(_rclpy.Context, self._context.handle))
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        # True when the executor sees this has been triggered but has not yet been handled
        self._executor_triggered = False

    def trigger(self) -> None:
        with self.__gc:
            self.__gc.trigger_guard_condition()

    @property
    def handle(self) -> _rclpy.GuardCondition:
        return self.__gc

    def destroy(self) -> None:
        self.handle.destroy_when_not_in_use()
