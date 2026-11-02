// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "signal_handler.hpp"

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/guard_condition.h>
#include <rcutils/logging_macros.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <csignal>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "context.hpp"
#include "guard_condition.hpp"

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif


namespace py = pybind11;

static bool trigger_guard_conditions();

namespace
{
#if defined(_WIN32)
HANDLE g_signal_handler_sem;
#elif defined(__APPLE__)
dispatch_semaphore_t g_signal_handler_sem;
#else  // posix
sem_t g_signal_handler_sem;
#endif

std::thread g_deferred_signal_handling_thread;

// All lifecycle entry points are serialized without retaining Python state.
// The asynchronous handler only uses lock-free atomics and the OS semaphore.
std::mutex g_signal_lifecycle_mutex;
std::atomic<bool> g_signal_handler_installed{false};
std::atomic<bool> g_signal_notifier_enabled{false};
std::atomic<unsigned int> g_signal_notifiers{0};
static_assert(std::atomic<bool>::is_always_lock_free, "signal flag must be lock-free");
static_assert(std::atomic<unsigned int>::is_always_lock_free, "signal readers must be lock-free");

void release_signal_caller(std::optional<py::gil_scoped_release> & release)
{
#if PY_VERSION_HEX >= 0x030D0000
  if (Py_IsInitialized() && PyThreadState_GetUnchecked() != nullptr) {
#else
  if (Py_IsInitialized() && PyGILState_Check()) {
#endif
    release.emplace();
  }
}

void
notify_signal_handler() noexcept
{
#if defined(_WIN32)
  if (!ReleaseSemaphore(g_signal_handler_sem, 1, NULL)) {
    return;
  }
#elif defined(__APPLE__)
  dispatch_semaphore_signal(g_signal_handler_sem);
#else  // posix
  if (-1 == sem_post(&g_signal_handler_sem)) {
    return;
  }
#endif
}

void
wait_for_signal()
{
#if defined(_WIN32)
  DWORD dw_wait_result = WaitForSingleObject(g_signal_handler_sem, INFINITE);
  switch (dw_wait_result) {
    case WAIT_ABANDONED:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals",
        "WaitForSingleObject() failed in wait_for_signal() with WAIT_ABANDONED: %s",
        GetLastError());
      break;
    case WAIT_OBJECT_0:
      // successful
      break;
    case WAIT_TIMEOUT:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() timedout out in wait_for_signal()");
      break;
    case WAIT_FAILED:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() failed in wait_for_signal(): %s", GetLastError());
      break;
    default:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() gave unknown return in wait_for_signal(): %s",
        GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_wait(g_signal_handler_sem, DISPATCH_TIME_FOREVER);
#else  // posix
  int s;
  do {
    s = sem_wait(&g_signal_handler_sem);
  } while (-1 == s && EINTR == errno);
#endif
}

void
setup_deferred_signal_handler()
{
  if (g_signal_handler_installed.load()) {
    return;
  }
#if defined(_WIN32)
  g_signal_handler_sem = CreateSemaphore(NULL, 0, 1, NULL);
  if (NULL == g_signal_handler_sem) {
    throw std::runtime_error("CreateSemaphore() failed in setup_wait_for_signal()");
  }
#elif defined(__APPLE__)
  g_signal_handler_sem = dispatch_semaphore_create(0);
  if (!g_signal_handler_sem) {
    throw std::runtime_error("dispatch_semaphore_create() failed");
  }
#else
  if (-1 == sem_init(&g_signal_handler_sem, 0, 0)) {
    throw std::runtime_error(std::string("sem_init() failed: ") + strerror(errno));
  }
#endif
  g_signal_handler_installed.store(true);
  try {
    g_deferred_signal_handling_thread = std::thread([]() {
        while (g_signal_handler_installed.load()) {
          wait_for_signal();
          if (!g_signal_handler_installed.load()) {
            break;
          }
          try {
            trigger_guard_conditions();
            rclpy::shutdown_contexts();
            rclpy::detail::invoke_signal_callbacks();
          } catch (const std::exception & error) {
            // Never unwind out of a native thread entry point.
            std::fprintf(stderr, "rclpy deferred signal handling failed: %s\n", error.what());
          } catch (...) {
            std::fputs("rclpy deferred signal handling failed: unknown exception\n", stderr);
          }
        }
      });
  } catch (...) {
    g_signal_handler_installed.store(false);
#if defined(_WIN32)
    CloseHandle(g_signal_handler_sem);
#elif defined(__APPLE__)
    dispatch_release(g_signal_handler_sem);
#else
    sem_destroy(&g_signal_handler_sem);
#endif
    throw;
  }
  // Publish only after both the semaphore and worker exist.
  g_signal_notifier_enabled.store(true);
}

void
teardown_deferred_signal_handler()
{
  if (!g_signal_handler_installed.load()) {
    return;
  }
  g_signal_notifier_enabled.store(false);
  // A reader that saw enabled=true must finish before the semaphore closes.
  // Readers entering later see false and never touch its OS allocation.
  while (g_signal_notifiers.load() != 0) {
    std::this_thread::yield();
  }
  g_signal_handler_installed.store(false);
  notify_signal_handler();
  g_deferred_signal_handling_thread.join();
#if defined(_WIN32)
  CloseHandle(g_signal_handler_sem);
#elif defined(__APPLE__)
  dispatch_release(g_signal_handler_sem);
#else
  if (-1 == sem_destroy(&g_signal_handler_sem)) {
    std::fputs("rclpy failed to destroy the signal semaphore\n", stderr);
  }
#endif
}

void notify_from_signal() noexcept
{
  const int saved_errno = errno;
  g_signal_notifiers.fetch_add(1);
  if (g_signal_notifier_enabled.load()) {
    notify_signal_handler();
  }
  g_signal_notifiers.fetch_sub(1);
  errno = saved_errno;
}

}  // namespace

#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE

#define SIGNAL_HANDLER_T struct sigaction

static SIGNAL_HANDLER_T
make_null_signal_handler()
{
  SIGNAL_HANDLER_T handler;
  memset(&handler, 0, sizeof(handler));
  return handler;
}

#define NULL_SIGNAL_HANDLER make_null_signal_handler()

#define DEFINE_SIGNAL_HANDLER(name) \
  static void _ ## name(int signum, siginfo_t * info, void * context); \
  static SIGNAL_HANDLER_T get_ ## name() { \
    SIGNAL_HANDLER_T handler; \
    memset(&handler, 0, sizeof(handler)); \
    sigemptyset(&handler.sa_mask); \
    handler.sa_sigaction = _ ## name; \
    handler.sa_flags = SA_SIGINFO; \
    return handler; \
  } \
  static bool is_ ## name(SIGNAL_HANDLER_T handler) { \
    return _ ## name == handler.sa_sigaction; \
  } \
  static void _ ## name(int signum, siginfo_t * info, void * context)

#define SIGNAL_HANDLER_ARGS signum, info, context

static void
call_signal_handler(
  SIGNAL_HANDLER_T handler, int signum,
  siginfo_t * siginfo, void * context)
{
  if (handler.sa_flags & SA_SIGINFO) {
    if (handler.sa_sigaction != NULL) {
      handler.sa_sigaction(signum, siginfo, context);
    }
  } else {
    if (handler.sa_handler != NULL &&    // Is set
      handler.sa_handler != SIG_DFL &&      // Is not default
      handler.sa_handler != SIG_IGN)      // Is not ignored
    {
      handler.sa_handler(signum);
    }
  }
}

static SIGNAL_HANDLER_T
install_signal_handler(int signum, SIGNAL_HANDLER_T handler)
{
  SIGNAL_HANDLER_T old_handler;
  if (sigaction(signum, &handler, &old_handler) != 0) {
    throw std::runtime_error("sigaction() failed");
  }
  return old_handler;
}

static SIGNAL_HANDLER_T query_signal_handler(int signum)
{
  SIGNAL_HANDLER_T handler;
  if (sigaction(signum, nullptr, &handler) != 0) {
    throw std::runtime_error("sigaction() query failed");
  }
  return handler;
}

#define is_null_signal_handler(handler) \
  (handler.sa_flags & SA_SIGINFO ? \
  !handler.sa_sigaction : \
  !handler.sa_handler)

#else

#ifdef _WIN32
#define SIGNAL_HANDLER_T _crt_signal_t
#else
#define SIGNAL_HANDLER_T sig_t
#endif  // _WIN32

#define NULL_SIGNAL_HANDLER NULL

#define DEFINE_SIGNAL_HANDLER(name) \
  static void _ ## name(int signum); \
  static SIGNAL_HANDLER_T get_ ## name() {return _ ## name;} \
  static bool is_ ## name(SIGNAL_HANDLER_T handler) { \
    return _ ## name == handler; \
  } \
  static void _ ## name(int signum)

#define SIGNAL_HANDLER_ARGS signum

static void
call_signal_handler(
  SIGNAL_HANDLER_T handler, int signum)
{
  if (handler != NULL && handler != SIG_DFL && handler != SIG_IGN) {
    handler(signum);
  }
}

static SIGNAL_HANDLER_T install_signal_handler(int signum, SIGNAL_HANDLER_T handler)
{
  const auto previous = std::signal(signum, handler);
  if (previous == SIG_ERR) {
    throw std::runtime_error("signal() failed");
  }
  return previous;
}

static SIGNAL_HANDLER_T query_signal_handler(int signum)
{
#ifdef _WIN32
  // Supported UCRT query; unlike PyOS_getsig's fallback it does not ignore
  // signals temporarily while looking up the previous handler.
  return install_signal_handler(signum, SIG_GET);
#else
  return PyOS_getsig(signum);
#endif
}

#define is_null_signal_handler(signal_handler) (NULL_SIGNAL_HANDLER == signal_handler)

#endif

// The async handler never reads mutable sigaction storage. Records are
// immutable and retired only by lifecycle callers after all readers leave.
struct PreviousSignalHandler
{
  explicit PreviousSignalHandler(SIGNAL_HANDLER_T value) : handler(value) {}
  const SIGNAL_HANDLER_T handler;
};

struct SignalHandlerState
{
  std::atomic<PreviousSignalHandler *> original{nullptr};
  std::atomic<unsigned int> readers{0};
  std::vector<std::unique_ptr<PreviousSignalHandler>> records;
  bool installed = false;  // protected by g_signal_lifecycle_mutex
  std::atomic<bool> armed{false};

  void publish(SIGNAL_HANDLER_T handler)
  {
    auto record = std::make_unique<PreviousSignalHandler>(handler);
    auto * current = record.get();
    records.push_back(std::move(record));
    original.store(current);
    if (readers.load() == 0) {
      records.erase(std::remove_if(records.begin(), records.end(),
        [current](const auto & value) {return value.get() != current;}), records.end());
    }
  }
};
static_assert(std::atomic<PreviousSignalHandler *>::is_always_lock_free,
  "original signal handler publication must be lock-free");

SignalHandlerState g_sigint;
SignalHandlerState g_sigterm;

DEFINE_SIGNAL_HANDLER(rclpy_sigint_handler)
{
  const int saved_errno = errno;
  g_sigint.readers.fetch_add(1);
  if (const auto * previous = g_sigint.original.load()) {
    call_signal_handler(previous->handler, SIGNAL_HANDLER_ARGS);
  }
#ifdef _WIN32
  // UCRT resets a delivered handler; CPython also reinstalls its predecessor.
  // Rearm after chaining, while uninstall waits for this reader to finish.
  if (g_sigint.armed.load()) {
    std::signal(SIGINT, get_rclpy_sigint_handler());
  }
#endif
  g_sigint.readers.fetch_sub(1);
  if (g_sigint.armed.load()) {
    notify_from_signal();
  }
  errno = saved_errno;
}

DEFINE_SIGNAL_HANDLER(rclpy_sigterm_handler)
{
  const int saved_errno = errno;
  g_sigterm.readers.fetch_add(1);
  if (const auto * previous = g_sigterm.original.load()) {
    call_signal_handler(previous->handler, SIGNAL_HANDLER_ARGS);
  }
#ifdef _WIN32
  if (g_sigterm.armed.load()) {
    std::signal(SIGTERM, get_rclpy_sigterm_handler());
  }
#endif
  g_sigterm.readers.fetch_sub(1);
  if (g_sigterm.armed.load()) {
    notify_from_signal();
  }
  errno = saved_errno;
}

static void register_sigint_signal_handler()
{
  if (g_sigint.installed) {
    g_sigint.armed.store(true);
    return;
  }
  // Publish before installing: the OS may deliver as soon as it sees our
  // function pointer. Other rclpy setters are serialized by the lifecycle lock.
  g_sigint.publish(query_signal_handler(SIGINT));
  g_sigint.armed.store(true);
  try {
    install_signal_handler(SIGINT, get_rclpy_sigint_handler());
    g_sigint.installed = true;
  } catch (...) {
    g_sigint.armed.store(false);
    throw;
  }
}

static void register_sigterm_signal_handler()
{
  if (g_sigterm.installed) {
    g_sigterm.armed.store(true);
    return;
  }
  g_sigterm.publish(query_signal_handler(SIGTERM));
  g_sigterm.armed.store(true);
  try {
    install_signal_handler(SIGTERM, get_rclpy_sigterm_handler());
    g_sigterm.installed = true;
  } catch (...) {
    g_sigterm.armed.store(false);
    throw;
  }
}

static void unregister_sigint_signal_handler()
{
  g_sigint.armed.store(false);
  while (g_sigint.readers.load() != 0) {
    std::this_thread::yield();
  }
  if (!g_sigint.installed) {
    return;
  }
  const auto current = query_signal_handler(SIGINT);
  bool restore = is_rclpy_sigint_handler(current);
#ifdef _WIN32
  // UCRT resets before entering a delivered handler. Its predecessor may
  // already have reinstalled itself while this unregister was draining it.
  restore = restore || current == SIG_DFL || current == g_sigint.original.load()->handler;
#endif
  if (restore) {
    install_signal_handler(SIGINT, g_sigint.original.load()->handler);
    g_sigint.installed = false;
  }
  // If another library chains to us, keep its handler and our immutable
  // predecessor intact. Async delivery must never try to edit this chain.
}

static void unregister_sigterm_signal_handler()
{
  g_sigterm.armed.store(false);
  while (g_sigterm.readers.load() != 0) {
    std::this_thread::yield();
  }
  if (!g_sigterm.installed) {
    return;
  }
  const auto current = query_signal_handler(SIGTERM);
  bool restore = is_rclpy_sigterm_handler(current);
#ifdef _WIN32
  restore = restore || current == SIG_DFL || current == g_sigterm.original.load()->handler;
#endif
  if (restore) {
    install_signal_handler(SIGTERM, g_sigterm.original.load()->handler);
    g_sigterm.installed = false;
  }
}

// This registry is read by the deferred worker, never by the asynchronous
// OS signal handler. Weak registrations do not pin abandoned Python guards;
// owning snapshots keep both the native guard and its context alive while
// rcl_trigger_guard_condition runs.
std::vector<std::weak_ptr<rcl_guard_condition_t>> g_guard_conditions;
std::mutex g_guard_conditions_mutex;

static bool
trigger_guard_conditions()
{
  std::vector<std::shared_ptr<rcl_guard_condition_t>> guards;
  {
    std::lock_guard<std::mutex> lock(g_guard_conditions_mutex);
    // Allocate before locking any weak resource. Last owners and their native
    // finalizers must be released after this registry mutex is unlocked.
    guards.reserve(g_guard_conditions.size());
    for (auto it = g_guard_conditions.begin(); it != g_guard_conditions.end(); ) {
      if (auto guard = it->lock()) {
        guards.push_back(std::move(guard));
        ++it;
      } else {
        it = g_guard_conditions.erase(it);
      }
    }
  }
  for (const auto & guard : guards) {
    rcl_ret_t ret = rcl_trigger_guard_condition(guard.get());
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals",
        "rcl_trigger_guard_condition() failed: %s", rcutils_get_error_string().str);
      rcl_reset_error();
    }
  }
  return !guards.empty();
}

namespace rclpy
{
void
register_sigint_guard_condition(const GuardCondition & guard_condition)
{
  auto guard = guard_condition.rcl_handle();
  std::lock_guard<std::mutex> lock(g_guard_conditions_mutex);
  g_guard_conditions.erase(
    std::remove_if(g_guard_conditions.begin(), g_guard_conditions.end(),
      [](const auto & weak) {return weak.expired();}), g_guard_conditions.end());
  for (const auto & weak : g_guard_conditions) {
    if (!weak.owner_before(guard) && !guard.owner_before(weak)) {
      throw py::value_error("Guard condition was already registered");
    }
  }
  g_guard_conditions.push_back(guard);
}

void
unregister_sigint_guard_condition(const GuardCondition & guard_condition)
{
  auto guard = guard_condition.rcl_handle();
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(g_guard_conditions_mutex);
    g_guard_conditions.erase(
      std::remove_if(g_guard_conditions.begin(), g_guard_conditions.end(),
        [&guard, &found](const auto & weak) {
          if (!weak.owner_before(guard) && !guard.owner_before(weak)) {
            found = true;
            return true;
          }
          return weak.expired();
        }), g_guard_conditions.end());
  }
  if (!found) {
    throw py::value_error("Guard condition was not registered");
  }
}

/// Enum to indicate which signal handlers to install.
enum class SignalHandlerOptions : int
{
  /// Install no signal handlers.
  No = 0,
  /// Install only a sigint handler.
  SigInt = 1,
  /// Install only a sigterm handler.
  SigTerm = 2,
  /// Install both a sigint and a sigterm handler.
  All = 3,
};

/// Install rclpy signal handlers.
/**
 * \param options rclpy.signals.SignalHandlerOptions integer value.
 */
void
install_signal_handlers(SignalHandlerOptions options)
{
  // Initialized on the first API use, after all registries in all translation
  // units. Its destructor therefore stops the worker before those registries.
  struct Cleanup
  {
    ~Cleanup()
    {
      try {
        std::optional<py::gil_scoped_release> release;
        release_signal_caller(release);
        std::lock_guard<std::mutex> lock(g_signal_lifecycle_mutex);
        unregister_sigint_signal_handler();
        unregister_sigterm_signal_handler();
        teardown_deferred_signal_handler();
      } catch (const std::exception & error) {
        std::fprintf(stderr, "rclpy signal cleanup failed: %s\n", error.what());
      }
    }
  };
  static Cleanup cleanup;
  std::optional<py::gil_scoped_release> release;
  release_signal_caller(release);
  std::lock_guard<std::mutex> lock(g_signal_lifecycle_mutex);
  setup_deferred_signal_handler();
  switch (options) {
    case SignalHandlerOptions::No:
      return;
    case SignalHandlerOptions::SigInt:
      register_sigint_signal_handler();
      return;
    case SignalHandlerOptions::SigTerm:
      register_sigterm_signal_handler();
      return;
    case SignalHandlerOptions::All:
      register_sigint_signal_handler();
      register_sigterm_signal_handler();
      return;
  }
}

/// Return the currently active signal handler options.
/**
 * \return rclpy.signals.SignalHandlerOptions converted to integer value.
 */
SignalHandlerOptions
get_current_signal_handlers_options()
{
  // conversion to SignalHandlerOptions value
  std::optional<py::gil_scoped_release> release;
  release_signal_caller(release);
  std::lock_guard<std::mutex> lock(g_signal_lifecycle_mutex);
  return SignalHandlerOptions{g_sigterm.armed.load() * 2 + g_sigint.armed.load()};
}

/// Uninstall the currently installed signal handlers.
void
uninstall_signal_handlers()
{
  std::optional<py::gil_scoped_release> release;
  release_signal_caller(release);
  std::lock_guard<std::mutex> lock(g_signal_lifecycle_mutex);
  unregister_sigint_signal_handler();
  unregister_sigterm_signal_handler();
  teardown_deferred_signal_handler();
}


}  // namespace rclpy


namespace rclpy
{
void
define_signal_handler_api(py::module m)
{
  m.def(
    "register_sigint_guard_condition", &rclpy::register_sigint_guard_condition,
    "Register a guard condition to be called on SIGINT.");
  m.def(
    "unregister_sigint_guard_condition", &rclpy::unregister_sigint_guard_condition,
    "Stop triggering a guard condition when SIGINT occurs.");
  m.def(
    "install_signal_handlers", &rclpy::install_signal_handlers,
    "Install rclpy signal handlers.");
  m.def(
    "get_current_signal_handlers_options", &rclpy::get_current_signal_handlers_options,
    "Get currently installed signal handler options.");
  m.def(
    "uninstall_signal_handlers", &rclpy::uninstall_signal_handlers,
    "Uninstall rclpy signal handlers.");
  py::enum_<SignalHandlerOptions>(
    m, "SignalHandlerOptions", "Enum with values: `ALL`, `SIGINT`, `SIGTERM`, `NO`.")
  .value("ALL", SignalHandlerOptions::All)
  .value("NO", SignalHandlerOptions::No)
  .value("SIGINT", SignalHandlerOptions::SigInt)
  .value("SIGTERM", SignalHandlerOptions::SigTerm);
}
}  // namespace rclpy
