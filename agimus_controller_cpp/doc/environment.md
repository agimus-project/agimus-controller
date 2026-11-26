# Environment (T0.1)

Recorded on 2026-11-25. Update this file when a dependency changes.

**Build and test with nix** (PR #334): the repository flake gets a C++ shell
(`nix develop .#cpp`, plain CMake) and a Python judge shell
(`nix develop .#py-judge`) with the code (branch `cpp-port/basics`). They
provide every dependency, including the ones the container below lacks
(doctest, colmpc, force_feedback_mpc). colcon still works, since the package
is plain CMake. The supported platforms are in PLAN.md, R9; T0.5 records
their dependency versions.

## C++ build environment: workspace_cadmium dev container

The container mounts the workspace at `/workspaces/workspace_cadmium`. ROS
distribution: lyrical, in `/opt/ros/lyrical`. OS: Ubuntu 26.04 (resolute),
glibc 2.43, GCC 15.2.0, CMake 4.2.3, Python 3.14.4.

| Dependency | Version | CMake config / location | Needed from |
|---|---|---|---|
| Eigen | 3.4.0 (`libeigen3-dev`) | `Eigen3::Eigen` | P1 |
| pinocchio | 4.1.0 | `/opt/ros/lyrical/lib/x86_64-linux-gnu/cmake/pinocchio`, target `pinocchio::pinocchio` | P1 |
| crocoddyl | 3.2.0 | `/opt/ros/lyrical/lib/x86_64-linux-gnu/cmake/crocoddyl` | P4 |
| coal | 3.0.3 | `/opt/ros/lyrical/lib/x86_64-linux-gnu/cmake/coal` | P3 |
| mim_solvers | 0.3.0 | built in this workspace: `install/mim_solvers/lib/cmake/mim_solvers` | P4 |
| example-robot-data | (workspace) | built in this workspace: `install/example_robot_data` | P3 |
| toml++ | 3.4.0 | `libtomlplusplus-dev` in the apt index, **not installed**; `tomlplusplus` in nixpkgs | P5 |
| doctest | **MISSING** | `doctest-dev` 2.4.12 is in the apt index, but the apt mirror (`repo.pal-local`) is unreachable from the container without the PAL network | P1 |
| colmpc (C++) | **MISSING** | no CMake config found | P6 |
| force_feedback_mpc (C++) | **MISSING** | no CMake config found | P8 |

Notes:

- **Pinocchio 4 moved its headers.** `pinocchio/spatial/se3.hpp` now lives
  under `include/pinocchio/deprecated/` (still on the include path); the new
  header is `pinocchio/spatial.hpp`. The code includes the new header when
  it exists (`__has_include`) and the old ones otherwise.
- glibc 2.43 still exports `__libc_malloc`, `__libc_free`, `__libc_calloc`,
  `__libc_realloc` and `__libc_memalign`, which the allocation hook relies on.
- pinocchio's and coal's CMake configs call `find_dependency(Boost)`, which
  warns under CMake 4 (CMP0167). The package sets the policy to NEW.
- `find` across `/` in the container also matches `/home/user/exchange/…`, a
  different mounted workspace. Ignore it.

## Build and test commands

From the container, in `/workspaces/workspace_cadmium`:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select agimus_controller_cpp \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select agimus_controller_cpp
colcon test-result --verbose
```

`BUILD_TESTING=ON` needs doctest (see above): install `doctest-dev` from a
network that reaches the PAL apt mirror, or use the nix C++ shell that comes
with the code (`nix develop .#cpp`, added on branch `cpp-port/basics`, not
in this plan-only branch). Without doctest, build the library alone
with `-DBUILD_TESTING=OFF`.

Benchmark and production builds: `-DCMAKE_BUILD_TYPE=Release
-DAGIMUS_CONTROLLER_RT_CHECKS=OFF` (see design.md, Performance).

## Python judge environment (T0.2)

Not set up yet. Planned: `nix develop` at the repository root (gepetto/nix
flake).
