# Local patches to this vendored copy of gz_ros2_control

Upstream: https://github.com/ros-controls/gz_ros2_control
Vendored at: `f538d25` ("Bump version of pre-commit hooks (#869) (#874)")

This directory used to be a nested git checkout, so `git status` in the parent
workspace showed it as a dirty submodule and its contents could not be pushed
with this project. The nested `.git` has been removed and the files are now
tracked normally by the workspace repo. That makes the copy pushable, but it
also means the local changes below are no longer distinguishable from upstream
by `git diff` — hence this note.

## `gz_ros2_control/CMakeLists.txt`

Installation of the **Ignition**-era target is guarded so it only happens off
Harmonic:

```cmake
if(NOT "$ENV{GZ_VERSION}" STREQUAL "harmonic")
install(TARGETS ign_ros2_control-system DESTINATION lib)
ament_export_libraries(ign_ros2_control-system gz_hardware_plugins)
endif()
```

Upstream installs `ign_ros2_control-system` unconditionally. On Harmonic
(`GZ_VERSION=harmonic`, which is what this workspace builds against) that
target is never created, so the unconditional `install(TARGETS ...)` fails the
build outright. The rest of the diff against upstream is comment and
blank-line churn only.

## Packages deliberately not built

`COLCON_IGNORE` files are present in `gz_ros2_control_demos/`,
`gz_ros2_control_tests/`, `ign_ros2_control/` and `ign_ros2_control_demos/`.
Only `gz_ros2_control` itself is needed by this project, and the Ignition
variants do not build against Harmonic.

## Re-syncing with upstream

Diff this tree against the upstream tag/commit above, re-apply the CMakeLists
guard, and keep the `COLCON_IGNORE` markers.
