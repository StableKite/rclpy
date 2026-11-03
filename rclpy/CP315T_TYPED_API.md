# CPython3.15t source qualification checkpoint

The real compiled extension and ROS API remain required. The66 production
Python files and2 native declarations pass strict Pyright with0 errors and
0 warnings on Windows/Linux. All387 SDK Python implementation files and all7
source generator/loader files pass the same strict analysis, without omissions
or disabled rules. The generated contracts retain original methods, defaults,
validation and all native bytes. SDKv10 is byte-identical to tested SDKv9.

Both complete56-file rclpy matrices pass per OS: automatic-GIL baseline and
experimental natural-GIL-OFF native module. All18 original generator tests and
2 new real CLI descriptor tests pass. Fresh Werror native builds and22 CTest
cases per build pass. An earlier original1ms Linux timer failure is retained
in the evidence, along with fixed balanced old/new controls; it is not erased.

The published native source retains its automatic-GIL declaration. Full native
acceptance and FT declaration publication, final exact-source matrices and
Python/native coverage, default-branch pip/SDK/wheel delivery,
required platform CI, upstream followups and independent acceptance remain open.
This is a source checkpoint, not a completed ROS or StableKite release.

Source and raw qualification index on the owner's PC:
A:/Codex/SK-Dependencies-A307-20261030/ros2-SDK10-source-draft-publication-20261103-v4/EVIDENCE.json

The runtime-hint repair resolves all18 previous cyclic-name failures. All66
rclpy modules/749 functions and387 SDK modules/7455 functions resolve with no
failures on both OS, natural GIL OFF. Three aliases return the original type
objects; their module body loads no ROS classes. All six affected whole test
files pass for both native policies per OS. The prior full56-file matrix is
retained; a final complete matrix on the latest source remains required.
