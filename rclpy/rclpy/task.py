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

from contextvars import ContextVar
from enum import Enum
import inspect
import sys
import threading
from typing import Any, Callable, Coroutine, Generator, Optional
import warnings
import weakref


def _fake_weakref():
    """Return None when called to simulate a weak reference that has been garbage collected."""
    return None


class _TaskCancelled(BaseException):
    """Internal cooperative cancellation at a rclpy Future await checkpoint."""


_current_task: ContextVar[Optional['Task']] = ContextVar('rclpy_current_task', default=None)


class FutureState(Enum):
    """States defining the lifecycle of a future."""

    PENDING = 'PENDING'
    CANCELLED = 'CANCELLED'
    FINISHED = 'FINISHED'


class Future:
    """Represent the outcome of a task in the future."""

    def __init__(self, *, executor=None):
        self._state = FutureState.PENDING
        # the final return value of the handler
        self._result = None
        # An exception raised by the handler when called
        self._exception = None
        self._exception_fetched = False
        # callbacks to be scheduled after this task completes
        self._callbacks = []
        # Lock for threadsafety
        self._lock = threading.Lock()
        # An executor to use when scheduling done callbacks
        self._executor = None
        self._set_executor(executor)

    def __del__(self):
        if self._exception is not None and not self._exception_fetched:
            print(
                'The following exception was never retrieved: ' + str(self._exception),
                file=sys.stderr)

    def __await__(self) -> Generator['Future', None, Optional[Any]]:
        task = _current_task.get()
        if task is not None and task.cancelled():
            raise _TaskCancelled()
        # Yield if the task is not finished
        if self._pending():
            # This tells the task to suspend until the future is done
            yield self
        if self._pending():
            raise RuntimeError('Future awaited a second time before it was done')
        # A different worker can cancel the task and complete this Future while
        # its coroutine is still entering await, so check even without a yield.
        if task is not None and task.cancelled():
            raise _TaskCancelled()
        return self.result()

    def _pending(self) -> bool:
        return self._state == FutureState.PENDING

    def cancel(self):
        """Request cancellation of the running task if it is not done already."""
        with self._lock:
            if not self._pending():
                return

            self._state = FutureState.CANCELLED
        self._schedule_or_invoke_done_callbacks()

    def cancelled(self):
        """
        Indicate if the task has been cancelled.

        :return: True if the task was cancelled
        :rtype: bool
        """
        return self._state == FutureState.CANCELLED

    def done(self):
        """
        Indicate if the task has finished executing.

        :return: True if the task is finished or raised while it was executing
        :rtype: bool
        """
        return self._state == FutureState.FINISHED

    def result(self):
        """
        Get the result of a done task.

        :raises: Exception if one was set during the task.

        :return: The result set by the task, or None if no result was set.
        """
        if self._exception:
            raise self.exception()
        return self._result

    def exception(self):
        """
        Get an exception raised by a done task.

        :return: The exception raised by the task
        """
        self._exception_fetched = True
        return self._exception

    def set_result(self, result):
        """
        Set the result returned by a task.

        :param result: The output of a long running task.
        """
        with self._lock:
            self._result = result
            self._state = FutureState.FINISHED

        self._schedule_or_invoke_done_callbacks()

    def set_exception(self, exception):
        """
        Set the exception raised by the task.

        :param result: The output of a long running task.
        """
        with self._lock:
            self._exception = exception
            self._exception_fetched = False
            self._state = FutureState.FINISHED

        self._schedule_or_invoke_done_callbacks()

    def _schedule_or_invoke_done_callbacks(self):
        """
        Schedule done callbacks on the executor if possible, else run them directly.

        This function assumes self._lock is not held.
        """
        with self._lock:
            executor = self._executor()
            callbacks = self._callbacks
            self._callbacks = []

        if executor is not None:
            # Have the executor take care of the callbacks
            for callback in callbacks:
                if isinstance(callback, Task):
                    executor._call_task_in_next_spin(callback)
                else:
                    executor.create_task(callback, self)
        else:
            # No executor, call right away
            for callback in callbacks:
                if isinstance(callback, Task):
                    warnings.warn(
                        'Dropping task awaiting future: '
                        'executor reference could not be resolved')
                    continue
                try:
                    callback(self)
                except Exception as e:
                    # Don't let exceptions be raised because there may be more callbacks to call
                    warnings.warn('Unhandled exception in done callback: {}'.format(e))

    def _set_executor(self, executor):
        """Set the executor this future is associated with."""
        with self._lock:
            if executor is None:
                self._executor = _fake_weakref
            else:
                self._executor = weakref.ref(executor)

    def add_done_callback(self, callback):
        """
        Add a callback to be executed when the task is done.

        Callbacks should not raise exceptions.

        The callback may be called immediately by this method if the future is already done.
        If this happens and the callback raises, the exception will be raised by this method.

        :param callback: a callback taking the future as an argument to be run when completed
        """
        invoke = False
        with self._lock:
            if not self._pending():
                executor = self._executor()
                if executor is not None:
                    executor.create_task(callback, self)
                else:
                    invoke = True
            else:
                self._callbacks.append(callback)

        # Invoke when not holding self._lock
        if invoke:
            callback(self)

    def _add_waiting_task(self, task: 'Task') -> None:
        """Schedule a task to resume when this future completes."""
        with self._lock:
            if not self._pending():
                executor = self._executor()
                if executor is not None:
                    executor._call_task_in_next_spin(task)
                else:
                    warnings.warn(
                        'Dropping task awaiting future: '
                        'executor reference could not be resolved')
            else:
                self._callbacks.append(task)

    def _remove_waiting_task(self, task: 'Task') -> None:
        """Release this future's cancelled/resumed task without invoking callbacks."""
        with self._lock:
            for index, callback in enumerate(self._callbacks):
                if callback is task:
                    del self._callbacks[index]
                    return

    def remove_done_callback(self, callback: Callable[['Future'], None]) -> bool:
        """
        Remove a previously-added done callback.

        Returns true if the given callback was found and removed.  Always fails if the Future was
        already complete.
        """
        with self._lock:
            try:
                self._callbacks.remove(callback)
            except ValueError:
                return False
            return True


class Task(Future):
    """
    Execute a function or coroutine.

    This executes either a normal function or a coroutine to completion. On completion it creates
    tasks for any 'done' callbacks.

    This class should only be instantiated by :class:`rclpy.executors.Executor`.
    """

    def __init__(self, handler, args=None, kwargs=None, executor=None):
        super().__init__(executor=executor)
        # _handler is either a normal function or a coroutine
        self._handler = handler
        # Arguments passed into the function
        if args is None:
            args = []
        self._args = args
        if kwargs is None:
            kwargs = {}
        self._kwargs = kwargs
        if inspect.iscoroutinefunction(handler):
            self._handler = handler(*args, **kwargs)
            self._args = None
            self._kwargs = None
        # True while the task is being executed
        self._executing = False
        # Lock acquired to prevent task from executing in parallel with itself
        self._task_lock = threading.RLock()
        # A waiting Future must release its task owner when the task is cancelled.
        self._waiting_on: Optional[Future] = None

    def __call__(self):
        """Run one serialized handler/coroutine step and retain queued resume requests."""
        # Another worker may receive a ready Future before the yielding worker
        # has released this task. Wait for that step; do not discard the wakeup.
        # RLock keeps direct same-thread reentry harmless.
        with self._task_lock:
            if not self._pending() or self._executing:
                return
            self._executing = True
            token = _current_task.set(self)
            try:
                self._clear_waiting_future()
                if inspect.iscoroutine(self._handler):
                    self._execute_coroutine_step(self._handler)
                else:
                    try:
                        self.set_result(self._handler(*self._args, **self._kwargs))
                    except Exception as e:
                        self.set_exception(e)
                    self._complete_task()
            finally:
                _current_task.reset(token)
                self._executing = False
                if self.cancelled():
                    self._close_cancelled_coroutine()

    def _execute_coroutine_step(self, coro: Coroutine) -> None:
        """Execute or resume a coroutine task."""
        try:
            result = coro.send(None)
        except _TaskCancelled:
            self._complete_task()
        except StopIteration as e:
            # The coroutine finished; store the result
            self.set_result(e.value)
            self._complete_task()
        except Exception as e:
            # The coroutine raised; store the exception
            self.set_exception(e)
            self._complete_task()
        else:
            # The coroutine yielded; suspend the task until it is resumed
            executor = self._executor()
            if executor is None:
                raise RuntimeError(
                    'Task tried to reschedule but no executor was set: '
                    'tasks should only be initialized through executor.create_task()')
            elif isinstance(result, Future):
                # Schedule the task to resume when the future is done
                self._add_resume_callback(result, executor)
            elif result is None:
                # The coroutine yielded None, schedule the task to resume in the next spin
                executor._call_task_in_next_spin(self)
            else:
                raise TypeError(
                    f'Expected coroutine to yield a Future or None, got: {type(result)}')

    def _add_resume_callback(self, future: Future, executor) -> None:
        future_executor = future._executor()
        if future_executor is None:
            # The future is not associated with an executor yet, so associate it with ours
            future._set_executor(executor)
        elif future_executor is not executor:
            raise RuntimeError('A task can only await futures associated with the same executor')

        # Register the task to resume when the future is done or cancelled
        self._waiting_on = future
        future._add_waiting_task(self)

    def _clear_waiting_future(self) -> None:
        future = self._waiting_on
        self._waiting_on = None
        if future is not None:
            future._remove_waiting_task(self)

    def _close_cancelled_coroutine(self) -> None:
        try:
            if inspect.iscoroutine(self._handler):
                self._handler.close()
        finally:
            self._complete_task()

    def _complete_task(self) -> None:
        """Cleanup after task finished."""
        self._clear_waiting_future()
        self._handler = None
        self._args = None
        self._kwargs = None
        self._notify_executor_task_done()

    def _notify_executor_task_done(self) -> None:
        # A task awaiting a Future may finish inside that Future's resume
        # callback, rather than through its executor's direct task dispatch.
        executor = self._executor()
        if executor is not None:
            notify = getattr(executor, '_on_task_done', None)
            if notify is not None:
                notify(self)

    def executing(self):
        """
        Check if the task is currently being executed.

        :return: True if the task is currently executing.
        :rtype: bool
        """
        return self._executing

    def cancel(self) -> None:
        super().cancel()
        if not self.cancelled():
            return
        try:
            # Cancellation is a request: never wait for a running user handler.
            # The active worker closes a suspended coroutine in its finally block.
            if self._task_lock.acquire(blocking=False):
                try:
                    if not self._executing:
                        self._close_cancelled_coroutine()
                finally:
                    self._task_lock.release()
        finally:
            self._notify_executor_task_done()
