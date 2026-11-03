# CPython 3.15t source qualification

This Jazzy source draft retains the real ROS APIs and adds a locally qualified
free-threaded module declaration. It is not a completed ROS or StableKite release.

The EventsExecutor node registry uses owning snapshots and serialized membership
changes. It releases its registry lock before Python callbacks or cross-executor
transfers. A per-Node reentrant lock serializes transfers between the Python and
native executors, so the final weak owner matches exactly one executor registry.

Native-held Python references participate in cyclic GC. Traversal chains the
original pybind11 managed-dictionary traversal, visits each actual owned reference
once, and tolerates an unconstructed holder. Finalization retires SDK callbacks
before Python dictionaries are cleared and preserves CPython resurrection rules.
The public module now declares `py::mod_gil_not_used()`; this declaration is
qualified by the substantive native repairs, not by an environment override.

## Local qualification

Windows x64 and Linux glibc x64 use CPython 3.15.2t. On each OS:

- The complete original registry of 56 test files passes on both the naturally
  GIL-OFF native build and the automatic-GIL control build. No original tests or
  timing thresholds were removed or weakened. Added tests cover real concurrent
  membership, executor transfers, Timer/ActionServer cycles and unconstructed GC.
- Both fresh native builds pass warnings-as-errors and all 22 native CTest cases.
- All 66 production Python files and two real native declarations pass strict
  Pyright 1.1.414 with zero errors and warnings. All 387 SDK Python implementation
  files and seven source generator/loader files also pass without exclusions.
- Runtime annotations resolve for all 749 rclpy functions and 7,455 SDK functions.
  Aliases retain actual native/Python type identity. Natural acceptance uses
  `sys._is_gil_enabled() == False`, `sys.flags.gil is None`, and no GIL override.
- Eight real Timer/ActionServer cycle cases and an unconstructed-holder case pass
  normally and in actual CPython development mode. A separate four-worker stress
  stage releases 128 cycles / 384 weak references with zero survivors or unraisable
  exceptions. Its unrelated debug preflight failure is retained; that entire
  packet is not described as passing.

All 18 original generator tests and two real CLI descriptor tests pass on both OS.
SDK v10 is byte-identical to tested SDK v9 across 887 files per OS; generated
methods, defaults, validation and 102 native converter files remain preserved.
Consumer type checks accept correct calls and reject all four wrong contracts.

## Evidence and open acceptance

Exact source review, before/after source provenance, raw commands and exit codes,
XML and native build logs are indexed on the owner's PC:

`A:/Codex/SK-Dependencies-A307-20261030/ros2-gc-transfer-source-review-20261103-v1/REVIEW.json`

Earlier failed builds and original timing failures remain in the evidence. Under
SysMonitor coverage, the latest Windows run still fails an original rate timing
test; its raw failure and measured coverage are retained separately from the
passing uninstrumented matrices. Coverage is not final acceptance.

Coverage gaps, separate C SDK coverage, final native acceptance, ordinary
default-branch pip installation and SDK/wheel delivery, all required platform CI,
upstream followups and independent acceptance remain open. Defaults and releases
are unchanged. Author qualification is not an independent audit.

## Measured coverage and clang-cl support

All 66 Python files were measured with no excluded lines or branches on both OS:
5,203 / 5,699 executable lines (91.2967%) and 1,266 / 1,636 branches (77.3839%).
The Linux instrumented full registry passes; Windows retains the original
SysMonitor rate timing failure described above.

Real native coverage now measures all 39 C++ implementation files and lists all
80 C++/header source files. The complete 56-file Python registry and all 22 native
CTest cases pass on each instrumented build with natural GIL OFF. Windows uses
the installed clang-cl 20 toolchain and LLVM source-based coverage; Linux uses
GCC/gcov with atomic profile updates. No native source files or branches are
excluded. External SDK code and the separate rcl C library are outside these
native metrics, and declaration-only headers without emitted regions are listed.

- Windows: 5,245 / 6,492 LLVM executable lines (80.7917%); 962 / 1,618 source
  branches (59.4561%).
- Linux: 3,081 / 3,818 GCC emitted source lines (80.6967%); 3,039 / 10,913
  compiler branch instances (27.8475%), retaining exceptional edges and duplicate
  template/inline instantiations. This denominator differs from LLVM's metric.

The clang-cl build uses the intended Clang warning groups through `/clang:`;
plain `-Wall` would select MSVC `/Wall` and enable unrelated warning groups.
Windows compiler atomics require no ELF libatomic dependency. Warnings remain
errors. The first failed mixed-toolchain/profile attempts remain in the evidence.

Raw native coverage, profiles, commands and complete reports:
`A:/Codex/SK-Dependencies-A307-20261030/ros2-native-coverage-export-{Windows,Linux}-20261103-v1/`.
These measurements are not a 100% coverage claim or final acceptance.
