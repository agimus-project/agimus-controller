# Environment (T0.1)

Recorded on 2026-11-25. Update this file when a dependency changes.

**Build and test with nix** (PR #334); colcon still works, since the
package is plain CMake. The supported platforms are in PLAN.md, R9; T0.5
records their dependency versions.

## C++ build environment: nix (preferred)

From the repository root (gepetto/nix pinned in `flake.lock`; everything
except our package comes from the gepetto and NixOS binary caches):

```bash
nix develop .#cpp                     # C++ only: pinocchio, Eigen, doctest, cmake
cmake -S agimus_controller_cpp -B build -DCMAKE_BUILD_TYPE=Debug   # configuration A
cmake --build build -j && (cd build && ctest)
nix build -L .#agimus-controller-cpp  # configuration B (RelWithDebInfo, -O3 -g) + tests, sandboxed
```

- **Do not use the default `nix develop`** for C++: it is the whole ROS
  stack, and if the pinned gepetto/nix revision is not in the cache it
  compiles ROS from source for hours.
- When bumping `flake.lock`, check first with
  `nix build --dry-run .#devShells.x86_64-linux.cpp` that only our own
  derivations would be built.
- Versions at gepetto/nix `a2acb49` (2026-11-23): pinocchio 4.1.0,
  crocoddyl 3.2.1, colmpc 0.4.0, mim-solvers 0.3.0, coal 3.0.4-dev,
  example-robot-data 5.0.0 (cached); force-feedback-mpc 1.0.1 (not
  cached: built locally); doctest and toml++ 3.4.0 from nixpkgs; GCC 15.3.
  colmpc and force-feedback-mpc are therefore available here, unlike in the
  container. Later phases add them to `package.nix` and to the shell.
- `nix flake check --no-build` fails on the ROS packages ("path … .drv is
  not valid"); this predates the C++ package.

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
network that reaches the PAL apt mirror, or use the nix C++ shell (`nix develop .#cpp`,
see the nix section above). Without doctest, build the library alone
with `-DBUILD_TESTING=OFF`.

Benchmark and production builds: `-DCMAKE_BUILD_TYPE=Release
-DAGIMUS_CONTROLLER_RT_CHECKS=OFF` (see design.md, Performance).

## Python judge environment (T0.2)

`nix develop .#py-judge` from the repository root: Python with the gepetto/nix
packages (all from the binary cache) and the `agimus_controller` package
imported from source (`PYTHONPATH`). It runs the Python tests and will run
the golden generator (T4.1; `tomli-w` writes the TOML fixtures).

```bash
nix develop .#py-judge --command bash -c 'cd agimus_controller && python -m pytest -q tests'
```

Result on 2026-11-25 (gepetto/nix `a2acb49`): **58 passed, 8 skipped**. The 8
skipped tests are the `agimus_franka_description` ones in `test_robot_models.py`
(they need xacro and ROS's `ament_index_python`).

| Package (Python) | Version | Same as the C++ nix shell? |
|---|---|---|
| Python | 3.14.7 | — |
| pinocchio | 4.1.0 | yes |
| crocoddyl | 3.2.1 | yes |
| mim-solvers | 0.3.0 | yes |
| colmpc | 0.4.0 | yes |
| coal | 3.0.4 | yes |
| numpy | 2.5.2 | — |
| force-feedback-mpc | not in the shell yet (not cached; needed in P8) | — |

Both sides come from the same gepetto/nix revision, so no minor version
differs and the golden tolerances in design.md apply as written. The
container differs only in patch versions (crocoddyl 3.2.0, coal 3.0.3).
