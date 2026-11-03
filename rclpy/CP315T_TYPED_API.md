# CPython 3.15t typed API checkpoint

The package retains its real compiled extension and existing public API. The
native .pyi files describe actual bound classes, enums and signatures; they do
not supply a Python implementation or remove the native build requirement.

Strict analysis covers all65 production Python files plus2 native declarations.
Both actual Windows and Linux compiled-companion/SDK layouts pass Pyright1.1.414
with0 errors/0 warnings and no disabled rules or omitted production files.
Positive consumer examples use real SDK message/service/action classes; four
wrong payload/response contracts are rejected. Existing py.typed and declarations
are preserved by actual ament_python_install_package on both operating systems.

The matching rosidl_python draft annotates296 generated Python files across34
IDL packages. Generation preserves original function/default/validation ASTs and
all non-Python bytes, including102 actual native converters per platform. All3
original producer test files/18 tests pass. Full strict analysis of the generated
implementation files is a separate open item.

Runtime qualification includes the full56-file frozen matrix and the12 whole
files affected by the latest public-contract refinement, with both native GIL
policies on both operating systems. Natural GIL OFF tests use no GIL override.
The public native module still keeps its original automatic-GIL declaration;
the experimental whole-module declaration remains unpublished pending review.

Full final coverage, default-branch pip/SDK delivery, required platform CI,
remaining native review and independent audit are open. This source checkpoint
is not a completed ROS or StableKite release.
