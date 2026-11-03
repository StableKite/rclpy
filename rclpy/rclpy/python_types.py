# Copyright 2026 StableKite contributors
# Licensed under the Apache License, Version 2.0.
"""Resolve cyclic annotation names to their original Python class/type objects.

The module body imports no ROS classes. Attribute access loads the real owning
module only when runtime annotation resolution requests that type.
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.executors import Executor as Executor
    from rclpy.node import Node as Node
    from rclpy.parameter import AllowableParameterValue as AllowableParameterValue

__all__ = ('AllowableParameterValue', 'Executor', 'Node')


def __getattr__(name: str) -> object:
    modules = {'AllowableParameterValue': 'parameter', 'Executor': 'executors', 'Node': 'node'}
    if name not in modules:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    from importlib import import_module
    return getattr(import_module('rclpy.' + modules[name]), name)


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
