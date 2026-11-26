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

## Platform matrix (T0.5)

Recorded on 2026-11-26. Cell format: version (source). ROS versions are the
release versions in `ros/rosdistro` (`<distro>/distribution.yaml`, master,
2026-11-26) and are what the robots install. ROS 2 Lyrical is the ROS
distribution for Ubuntu 26.04 (resolute): its `release_platforms` lists
`ubuntu: resolute`.

| Dependency | NixOS (gepetto/nix `a2acb49`) | Ubuntu 22.04 / Humble | Ubuntu 24.04 / Jazzy | Ubuntu 26.04 / Lyrical | Proposed minimum |
|---|---|---|---|---|---|
| Eigen | 3.4.1 (nixpkgs `eigen`) | 3.4.0 (apt `libeigen3-dev`) | 3.4.0 (apt `libeigen3-dev`) | 3.4.0 (apt `libeigen3-dev`) | 3.3 (as now; all platforms have 3.4) |
| pinocchio | 4.1.0 (gepetto/nix) | 4.1.0 (`ros-humble-pinocchio`) | 4.1.0 (`ros-jazzy-pinocchio`) | 4.1.0 (`ros-lyrical-pinocchio`) | 4.1.0 (see flags) |
| crocoddyl | 3.2.1 (gepetto/nix) | 3.2.1 (`ros-humble-crocoddyl`) | 3.2.1 (`ros-jazzy-crocoddyl`) | 3.2.0 (`ros-lyrical-crocoddyl`) | 3.2.0 |
| coal | 3.0.4 (gepetto/nix) | 3.0.3 (`ros-humble-coal`); also hpp-fcl 2.4.5 (`ros-humble-hpp-fcl`) | 3.0.3 (`ros-jazzy-coal`); also hpp-fcl 2.4.5 | 3.0.3 (`ros-lyrical-coal`); no hpp-fcl | 3.0.3 |
| mim_solvers | 0.3.0 (gepetto/nix `mim-solvers`) | not packaged | not packaged | not packaged | 0.3.0 (source build) |
| colmpc | 0.4.0 (gepetto/nix) | not packaged | not packaged | not packaged | 0.4.0 (source build) |
| force_feedback_mpc | 1.0.1 (gepetto/nix `force-feedback-mpc`, not cached) | not packaged | not packaged | not packaged | 1.0.1 (source build) |
| example-robot-data | 5.0.0 (gepetto/nix) | 7.0.0 (`ros-humble-example-robot-descriptions`) | 7.0.0 (`ros-jazzy-example-robot-descriptions`) | 7.0.0 (`ros-lyrical-example-robot-descriptions`) | 5.0.0 |
| doctest | 2.5.3 (nixpkgs) | 2.4.8 (apt `doctest-dev`) | 2.4.11 (apt `doctest-dev`) | 2.4.12 (apt `doctest-dev`) | 2.4.8 |
| toml++ | 3.4.0 (nixpkgs `tomlplusplus`) | not packaged | 3.4.0 (apt `libtomlplusplus-dev`) | 3.4.0 (apt `libtomlplusplus-dev`) | 3.4.0 |
| CMake | 4.4.2 (nixpkgs) | 3.22.1 (apt `cmake`) | 3.28.3 (apt `cmake`) | 4.2.3 (apt `cmake`) | 3.22 (as now) |
| GCC | 15.3.0 (nixpkgs) | 11.2.0 (apt `g++`, default) | 13.2.0 (apt `g++`, default) | 15.2.0 (apt `g++`, default) | 11 (C++17) |

Notes on the table:

- The ROS packages are the `ros2-gbp` releases of the upstream projects. The
  ROS package for example-robot-data is named `example_robot_descriptions`;
  it installs the `example-robot-data` CMake project. Its version (7.0.0) is
  not on the same scale as the nix one (5.0.0), so the two numbers are not
  comparable. Not checked: whether 5.0.0 and 7.0.0 install the same
  `example-robot-data` CMake config and robot set.
- Ubuntu itself has no pinocchio, crocoddyl, coal or hpp-fcl source package
  (Launchpad query, jammy, noble, resolute). robotpkg was not consulted.
- Nix values were read with `nix eval` on the gepetto/nix inputs of
  `flake.lock` and from `nix develop .#cpp` (Eigen, CMake, GCC, doctest,
  coal 3.0.4-dev, pinocchio, example-robot-data). The `cpp` shell does not
  yet contain crocoddyl, mim-solvers, colmpc, force-feedback-mpc or
  toml++; their versions are the packages in gepetto/nix.
- The container (Lyrical) has crocoddyl 3.2.0 and coal 3.0.3, the same as the
  Lyrical column.

### Minimum versions and flags

- The minimum is the oldest version found on any platform where the
  dependency exists. Use it in `find_package(<name> <min> REQUIRED)`; keep
  `Eigen3 3.3` and `cmake_minimum_required(3.22)` as they are.
- **coal vs hpp-fcl.** Every platform has `coal` (3.0.x), so
  `find_package(coal REQUIRED)` works everywhere, including Humble and Jazzy.
  Humble and Jazzy also ship the old `hpp-fcl` 2.4.5, which pinocchio 4.1.0
  does not use here (the ROS pinocchio package depends on coal). Do not add
  an hpp-fcl fallback. Lyrical ships only coal.
- **pinocchio 4.1.0** is the only version seen. Setting the minimum to 4.1.0
  is stricter than the port needs (pinocchio 3+), but no platform offers an
  older one, and the code already handles the pinocchio 4 header move.
- **crocoddyl 3.2.0** satisfies the 3.x API that the port uses.
- **doctest 2.4.8** (Humble) is old but fine for the tests.
- **Too old for the port:** nothing found. GCC 11.2 (Humble) supports C++17
  fully. CMake 3.22 is the Humble version and is already the project minimum.
- **mim_solvers, colmpc, force_feedback_mpc** have no ROS or apt package, so
  their minimums (0.3.0, 0.4.0, 1.0.1) come from nix only. Not verified
  against older upstream tags: that those versions are the oldest that work.

### Missing

- Humble (Ubuntu 22.04): toml++ (not packaged in jammy), mim_solvers, colmpc,
  force_feedback_mpc.
- Jazzy (Ubuntu 24.04): mim_solvers, colmpc, force_feedback_mpc.
- Lyrical (Ubuntu 26.04): mim_solvers, colmpc, force_feedback_mpc.
- NixOS: nothing missing. force-feedback-mpc is not in the binary cache and
  is built locally.
- None of mim_solvers, colmpc and force_feedback_mpc is in any of the three
  `distribution.yaml` files.

### CI proposal (for approval)

The repository already has `.github/workflows/nix.yml` (ROS builds through
nix) and `ci_ros2.yml` (industrial_ci on humble, jazzy, kilted, rolling,
with `UPSTREAM_WORKSPACE` for colmpc, example-robot-data and mim_solvers).

1. **nix job:** `nix build -L .#agimus-controller-cpp` on `ubuntu-24.04`, with
   `cachix/install-nix-action` and the `gepetto` and `ros` caches, as in
   `nix.yml`. This builds with configuration B and runs the tests in the
   sandbox. Optionally add `nix develop .#cpp` with a Debug build
   (configuration A).
2. **ROS jobs, one per Ubuntu release:** industrial_ci (as in `ci_ros2.yml`)
   with `ROS_DISTRO` set to humble (`ros:humble-ros-base`, Ubuntu 22.04),
   jazzy (`ros:jazzy-ros-base`, 24.04) and lyrical (`ros:lyrical-ros-base`,
   26.04). The existing matrix would add `lyrical` and may drop `kilted` and
   `rolling` for the C++ package. The lyrical image exists on Docker Hub
   (checked); humble and jazzy are long-standing images, not re-checked.
   - pinocchio, crocoddyl, coal and example-robot-data come from the ROS
     binaries; Eigen, doctest, CMake and GCC from Ubuntu apt (`rosdep` or
     `ADDITIONAL_DEBS`).
   - mim_solvers, colmpc and force_feedback_mpc from `UPSTREAM_WORKSPACE`
     (source build, as today for mim_solvers and colmpc). This slows each
     job; a cached docker image with them pre-built is a later option.
   - toml++: apt `libtomlplusplus-dev` on jazzy and lyrical. On humble there
     is no package: either fetch the single header in the job, or add
     toml++ to `UPSTREAM_WORKSPACE` (it is a CMake project), or use
     `FetchContent` in `CMakeLists.txt` with a `find_package` first.

Sources consulted (2026-11-26): `raw.githubusercontent.com/ros/rosdistro/master/{humble,jazzy,lyrical}/distribution.yaml`;
`ros2-gbp/example-robot-descriptions-release` `package.xml` (humble branch);
Launchpad `getPublishedSources` for eigen3, doctest, tomlplusplus, cmake,
gcc-defaults, gcc-11/13/14/15, pinocchio, coal, hpp-fcl, crocoddyl in jammy,
noble and resolute (packages.ubuntu.com was too slow to use); Docker Hub tag
API for `ros:lyrical-ros-base`; `nix eval` and `nix develop .#cpp` on this
repository's `flake.lock`.

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
