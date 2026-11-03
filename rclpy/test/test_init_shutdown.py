# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import rclpy
from rclpy import signals
from rclpy.exceptions import NotInitializedException
from rclpy.signals import SignalHandlerOptions


def test_init():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    assert context.ok()
    rclpy.shutdown(context=context)


def test_init_with_unknown_ros_args():
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    context = rclpy.context.Context()
    unknown_ros_args_error_pattern = r'\[\'unknown\'\]'
    with pytest.raises(_rclpy.UnknownROSArgsError, match=unknown_ros_args_error_pattern):
        rclpy.init(context=context, args=['--ros-args', 'unknown'])


def test_init_with_non_utf8_arguments():
    context = rclpy.context.Context()
    # Embed non decodable characters e.g. due to
    # wrong locale settings.
    # See PEP-383 for further reference.
    args = ['my-node.py', 'Ragnar\udcc3\udcb6k']
    with pytest.raises(UnicodeEncodeError):
        rclpy.init(context=context, args=args)


def test_init_shutdown_sequence():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    assert context.ok()
    rclpy.shutdown(context=context)
    context = rclpy.context.Context()  # context cannot be reused but should not interfere
    rclpy.init(context=context)
    assert context.ok()
    rclpy.shutdown(context=context)

    # global
    rclpy.init()
    assert rclpy.ok()
    rclpy.shutdown()
    rclpy.init()
    assert rclpy.ok()
    rclpy.shutdown()


def test_double_init():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(RuntimeError):
            rclpy.init(context=context)
    finally:
        rclpy.shutdown(context=context)


def test_double_shutdown():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    assert context.ok()
    rclpy.shutdown(context=context)
    with pytest.raises(RuntimeError):
        rclpy.shutdown(context=context)


def test_create_node_without_init():
    context = rclpy.context.Context()
    with pytest.raises(NotInitializedException):
        rclpy.create_node('foo', context=context)


def test_init_with_domain_id():
    rclpy.init(domain_id=123)
    assert rclpy.get_default_context().get_domain_id() == 123
    rclpy.shutdown()
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=123)
    assert context.get_domain_id() == 123
    rclpy.shutdown(context=context)


def test_signal_handlers():
    rclpy.init()
    assert SignalHandlerOptions.ALL == signals.get_current_signal_handlers_options()
    rclpy.shutdown()
    assert SignalHandlerOptions.NO == signals.get_current_signal_handlers_options()


def test_init_with_invalid_domain_id():
    with pytest.raises(RuntimeError):
        rclpy.init(domain_id=-1)


def test_global_executor_shutdown_and_reinitialization():
    rclpy.init()
    try:
        first = rclpy.get_global_executor()
    finally:
        rclpy.shutdown()

    rclpy.init()
    try:
        second = rclpy.get_global_executor()
        assert second is not first
    finally:
        rclpy.shutdown()


def test_python_annotation_alias_body_preserves_lazy_imports():
    import runpy
    import sys
    from rclpy import python_types

    before = {name for name in sys.modules if name == 'rclpy' or name.startswith('rclpy.')}
    body = runpy.run_path(python_types.__file__)
    after = {name for name in sys.modules if name == 'rclpy' or name.startswith('rclpy.')}
    assert after == before
    assert set(body['__all__']) == {'Node', 'Executor', 'AllowableParameterValue'}


def test_python_annotation_aliases_retain_original_type_identity():
    from rclpy import python_types
    from rclpy.executors import Executor
    from rclpy.node import Node
    from rclpy.parameter import AllowableParameterValue

    assert python_types.Node is Node
    assert python_types.Executor is Executor
    assert python_types.AllowableParameterValue is AllowableParameterValue
    assert set(python_types.__all__) <= set(dir(python_types))
    with pytest.raises(AttributeError, match='not_a_python_type'):
        getattr(python_types, 'not_a_python_type')


def test_public_cyclic_type_hints_resolve_real_ros_types():
    from importlib import import_module
    from typing import get_type_hints

    functions = {
        'rclpy': ('get_global_executor', 'create_node', 'spin_once', 'spin', 'spin_until_future_complete'),
        'rclpy.exceptions': ('InvalidParameterValueException.__init__',),
        'rclpy.executors': ('TaskData.__init__', 'Executor.add_node', 'Executor.remove_node',
                            'Executor.get_nodes', 'Executor._make_handler',
                            'Executor._wait_for_ready_callbacks', 'Executor.wait_for_ready_callbacks'),
        'rclpy.logging_service': ('LoggingService.__init__',),
        'rclpy.qos_overriding_options': ('_declare_qos_parameters',),
        'rclpy.time_source': ('TimeSource.__init__', 'TimeSource.attach_node', 'TimeSource._get_node'),
    }
    count = 0
    for module_name, names in functions.items():
        module = import_module(module_name)
        for name in names:
            value = module
            for attribute in name.split('.'):
                value = getattr(value, attribute)
            hints = get_type_hints(value)
            assert hints, module_name + '.' + name
            count += 1
    assert count == 18
