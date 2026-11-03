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
import gc
import threading
import weakref

import pytest
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.clock_type import ClockType
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as native
from rclpy.time import Time


def make_clock():
    clock = ROSClock()
    clock._set_ros_time_is_active(True)
    return clock


def threshold():
    return JumpThreshold(min_forward=Duration(nanoseconds=1), min_backward=None)


def test_expired_jump_handle_does_not_leave_dangling_userdata():
    clock = make_clock()
    callback_calls = []
    for index in range(32):
        handle = clock.create_jump_callback(
            threshold(), post_callback=lambda jump: callback_calls.append(jump))
        reference = weakref.ref(handle)
        del handle
        gc.collect()
        assert reference() is None
        clock.set_ros_time_override(Time(nanoseconds=index + 1))
    assert callback_calls == []


def test_unregister_during_pre_callback_preserves_other_callbacks():
    clock = make_clock()
    called = []
    handle = clock.create_jump_callback(
        threshold(), pre_callback=lambda: (called.append('pre'), handle.unregister()),
        post_callback=lambda jump: called.append('unexpected post'))
    other = clock.create_jump_callback(
        threshold(), post_callback=lambda jump: called.append('other post'))
    clock.set_ros_time_override(Time(nanoseconds=1))
    assert called == ['pre', 'other post']
    other.unregister()


def test_unregister_is_a_single_transaction_across_threads():
    clock = make_clock()
    called = []
    handle = clock.create_jump_callback(
        threshold(), post_callback=lambda jump: called.append(jump))
    barrier = threading.Barrier(8)

    def unregister(_):
        barrier.wait(timeout=10)
        for _ in range(32):
            handle.unregister()

    with ThreadPoolExecutor(max_workers=8) as executor:
        list(executor.map(unregister, range(8)))
    clock.set_ros_time_override(Time(nanoseconds=1))
    assert called == []


def test_callback_exception_propagates_without_unwinding_through_rcl():
    clock = make_clock()

    def fail():
        raise ValueError('jump callback regression')

    handle = clock.create_jump_callback(threshold(), pre_callback=fail)
    with pytest.raises(ValueError, match='jump callback regression'):
        clock.set_ros_time_override(Time(nanoseconds=1))
    assert clock.now().nanoseconds == 1
    handle.unregister()
    clock.set_ros_time_override(Time(nanoseconds=2))


def test_timer_lifetime_changes_during_jump_do_not_mutate_sdk_traversal():
    clock = make_clock()
    context = Context()
    context.init(args=[], domain_id=219)
    timers = []
    try:
        timer = native.Timer(clock.handle, context.handle, 1000, True)
        timers.append(timer)

        def replace_timers():
            for timer in timers:
                timer.destroy_when_not_in_use()
            timers.clear()
            transient = native.Timer(clock.handle, context.handle, 1000, True)
            transient.destroy_when_not_in_use()

        handle = clock.create_jump_callback(threshold(), pre_callback=replace_timers)
        clock.set_ros_time_override(Time(nanoseconds=1))
        handle.unregister()
        assert timers == []
    finally:
        for timer in timers:
            timer.destroy_when_not_in_use()
        context.shutdown()


def test_ros_wait_checks_target_reached_before_wait_registration():
    clock = make_clock()
    clock.set_ros_time_override(Time(nanoseconds=10))
    event = native.ClockEvent()
    stuck = False
    with ThreadPoolExecutor(max_workers=1) as executor:
        waiter = executor.submit(
            event.wait_until_ros, clock.handle, native.rcl_time_point_t(9, ClockType.ROS_TIME))
        try:
            waiter.result(timeout=2)
        except TimeoutError:
            stuck = True
            event.set()
            waiter.result(timeout=2)
    assert not stuck
