# Ledger

One entry per finished task (format in supervision.md). Rulings recorded
here bind later tasks; rulings that change the design are also edited into
design.md.

## T0.1 C++ environment: done
- Recorded in environment.md.
- Open: doctest, colmpc and force_feedback_mpc are missing in the container
  (user decision needed; colmpc blocks P6, force_feedback_mpc blocks P8).

## T1.1 Skeleton, T1.2 RT harness, T2.1–T2.5 data types: done (pending review)
- Verified in a scratch build in the container (`/tmp/agimus_cpp_verify`)
  against a throw-away doctest 2.4.11 shim, because doctest-dev is missing.
  The shim's `doctest_discover_tests` registers one ctest per executable.
  Result: 7/7 ctests, 66/66 test cases pass, no compiler warnings.
- `AGIMUS_RT_ABORT_ON_ALLOC=1 test_rt_harness -tc="RtHarness: vector growth
  is detected"` exits with SIGABRT (134), as required by T1.2.
- Rulings:
  - `EIGEN_NO_AUTOMATIC_RESIZING` dropped: Eigen 3.4 ignores it on plain
    assignment (only `PlainObjectBase::_resize_to_match` checks it); found
    by the harness's own negative test. A resize under the guard is still
    caught through the reallocation. rt_safety.md updated.
  - An Eigen allocation under the guard aborts the test executable instead
    of being counted (Eigen's check fires before the hook sees it).
  - GCC elides `new`/`delete` pairs whose pointer does not escape: negative
    tests write pointers and results to `volatile` globals.
  - `WeightedTrajectoryPoint::w_q` has size nv, not nq (Python test_buffer
    and the tangent-space state residual). design.md fixed.
  - `interpolate_weights` returns a `Status` (`kSizeMismatch`,
    `kBufferFull`), appends missing union frames to `out` with identity
    pose, and zeroes the weights of `out` frames in neither input.
  - `TrajectoryBuffer::push_back` also returns `kSizeMismatch` for a point
    not sized like the prototype, because copying it would reallocate.
  - Added `same_sizes`, `copy_results` and `to_string(Status)` as `@rt` free
    functions.
- Deviations: the Python tests "horizon raises on empty buffer" and "pop"
  become `has_horizon() == false` and `back()`.

## Review of the plan (PR #334), 2026-11-25
- Applied (docs): `EIGEN_INITIALIZE_MATRICES_BY_NAN` in RT-check builds;
  test configurations A (debug), B (`-O3`), C (`-O3 -march=native`, whole
  stack only, at phase ends); time as `std::chrono::nanoseconds` from
  `steady_clock`, converted to `double` only at boundaries; zero weight
  disables the cost (B10, with the `changeCostStatus` allocation found in
  crocoddyl 3.2); bug-report rule and T0.4 bug audit; B4 and B7 explained,
  B7 redesigned (one-argument `circularAppend` allocates every tick);
  jrl-cmakemodules update in T5.1; crocoddyl_yaml type registry (T5.5),
  colmpc and force-feedback types moved to `CPP/`; per-node allocation
  for OpenMP, measured first (T10.4); benchmarks on a desktop CPU.
- Applied (code): `time`, `dt`, `timesteps`, `total_time`,
  `max_solve_time` and the debug timers are `std::chrono::nanoseconds`;
  the RT-check definitions are `$<BUILD_INTERFACE:...>`, so the installed
  target does not inherit them. Configurations A and B green (67 ctests),
  in the container (doctest shim) and in a nix shell (real doctest).
- Open, for the user: B4 (a `control_grav` residual with a reference is
  not planned); B7 (keep the rolling buffer or update every node).

## Ruling R2 revised: crocoddyl-toml, 2026-11-25
- After the gepetto maintainers' TOML comment on PR #334, the user decided:
  a new crocoddyl-toml library in its own repository; a TOML-native format
  (not crocoddyl_yaml's schema); validation by typed C++ structs;
  crocoddyl_yaml dropped (not upgraded, no parity tests against it).
- Supersedes the review's crocoddyl_yaml items (jrl-cmakemodules update,
  type registry inside crocoddyl_yaml): the registry moves to crocoddyl-toml.
- crocoddyl_yaml (PepMS, December 2024) is acknowledged in
  crocoddyl_toml.md as the design crocoddyl-toml starts from; its defects
  became requirements.
- Choices made while writing crocoddyl_toml.md (the user may veto):
  the agimus keys move into an `[agimus]` table of the same file (they were
  a separate file only because the crocoddyl_yaml schema was frozen);
  colmpc dynamics are chosen explicitly (`colmpc_free_fwd`), not inferred
  from the residuals as Python does; golden fixtures are TOML instead of
  YAML (no yaml-cpp dependency left).
- Repository: github.com/loco-3d/crocoddyl-toml (created empty by the
  user). It follows the Gepetto packaging (jrl-cmakemodules as an installed
  package, `package.xml`, gepetto/nix `mkFlakoboros` flake, example-adder's
  CI), so R6's plain CMake applies to `CPP/` only. Its flake needs a
  `pkgs/crocoddyl-toml.nix` in gepetto/nix (pull request, user approval).
- Open: the license (none yet; crocoddyl is BSD-3-Clause); doctest or
  Boost.Test (loco-3d maintainers); Python bindings later.

## Rulings B4 and B7, 2026-11-25
- B7: keep the rolling buffer (real-time rolling, design.md B7 note).
  A reset was designed, then dropped: "forget about the reset" (user). No
  `Mpc::reset`, `Ocp::reset`, `WarmStart::reset` or `TrajectoryBuffer::clear`.
- B4: the user is not sure; the plan keeps parity (no residual with a
  reference). Revisit if a demo needs torque tracking around gravity.

## Review of PR #334 by Kotochleb, 2026-11-25/26 (with the user's replies)
- Time: solver-side values (`OcpParams` dt, timesteps, total_time,
  max_solve_time; crocoddyl; crocoddyl-toml's `build_model`) are `double`
  seconds; agimus' external time stamps and measured durations stay
  `std::chrono`. Code follow-up: T2.6.
- crocoddyl-toml: one schema file (JSON Schema subset) per type; typed C++
  parsing and registration generated at build time, ROS-message style; a
  generated editor schema for TOML language servers. Downstream types use
  the same CMake function (endorsed by the user). T5.2 is now H.
- crocoddyl_toml.md split: generic library only; the agimus side moved to
  the new ocp_files.md, which `CT/` briefs do not include.
- Debug data and phase timers are opt-in (`use_debug_data`), as in Python.
- Benchmarking: `steady_clock`; `nice -n -20` (or `chrt -f`); an OpenMP
  affinity/binding step; optional GCC/Clang/oneAPI comparison later
  (T10.8); rolling against full update (T10.2).
- Rolling buffer: the reviewer asks that it be allowed only when every node
  has the same dt; the plan already requires every dt factor to be 1.
- Environment: build and test with nix; colcon still works. New ruling R9
  (user): NixOS and Ubuntu 22.04/24.04/26.04; dependencies follow those
  platforms (task T0.5); bump to Eigen 5 and newer libraries after the
  port.
- rt_safety: configuration B is `RelWithDebInfo` (accepted suggestion) with
  `-O3 -g` set explicitly (CMake's default is `-O2`); it uses more compile
  RAM, so the 2 GB rule is checked there. Configuration C (Release,
  `-march=native`, whole stack) is the last stage and needs its own
  environment. Code follow-up on `cpp-port/basics`: `package.nix` builds
  configuration B.
## T0.2 Python judge: done
- `nix develop .#py-judge` (flake devShell): 58 passed, 8 skipped (franka
  description tests). Versions identical to the C++ nix shell (gepetto/nix
  `a2acb49`); see environment.md.
- force-feedback-mpc (Python) is not in the shell yet: add it in P8 (it is
  built locally, not cached).

## T0.4 Bug audit of the Python sources: done, rulings pending
16 findings at `630372d` (A1–A16); the supervisor re-read A1, A4, A6 and A13
in the code and confirms them. B1–B10 were re-checked and are accurate.
Proposed handling (the user decides; accepted fixes become B-rows in
design.md with a test):

| Id | Where | Bug | Proposed |
|---|---|---|---|
| A1 | ROS:417-425, mpc.py:38 | `buffer_has_enough_data(1)` true at N points, `run` needs N+1: the controller stalls for good when the publisher stops | C++ uses integer counts (`has_horizon`); report upstream (ROS node not ported) |
| A2 | ROS:131-163 | `q0` from `/joint_states` never passed: locked joints at neutral | report upstream; C++ `RobotModels` keeps `q0` (T3.1) |
| A3 | mpc.py:32 | `current_time_ns` ignored | drop the parameter from `Mpc::run` (B11) |
| A4 | ocp_croco_goal_reaching.py:206 | terminal cost uses the last running node's `ee_id` | fix in T4.3 (B12) |
| A5 | force_feedback.py:185-214 | `force_lb/ub` ignored without constraints | fix in P8 (B13) |
| A6 | ocp_croco_goal_reaching.py:18-20,82-84 | state weights sized `nq+nv`, not `ndx` | fix in T4.3 (B14); panda has nq == nv, so G5 is unaffected |
| A7 | ws_shift_force_feedback.py:192-195 | dict used as a `Force` | fix in P8 (B15) |
| A8 | force_feedback.py:40,149-150 | default tuple used as a numpy mask | fix in P8 (B16) |
| A9 | ocp_croco_generic.py:700, force_feedback.py:104 | `constraints is None` never true: empty manager always attached | C++ attaches a manager only with constraints; check parity in T7.4 |
| A10 | ocp_croco_generic.py:571-574 | `update` with no / exp activation: weights dropped or crash | setup error in T7.1 (B17) |
| A11 | ocp_croco_generic.py:227,281,335,400 | `*Static` classes reuse the non-static `class_` (`as_dict`) | not ported; report upstream |
| A12 | mpc.py:76-77, ocp_base_croco.py:187-189 | delay compensation integrates node 0's step but advances time by `dt` | `Mpc::integrate` advances time by `timesteps[0]`, the step it integrates (extends B1) |
| A13 | warm_start_shift_previous_solution.py:93-104 | terminal state never shifted when its step > dt | fix in T4.4 (B18); G3 then differs at `xs[N]` only |
| A14 | robot_model.py:50-62,346-351 | armature validated and ordered by joint name, used by DoF | fix in T3.1 follow-up (B19): map names to `idx_v` |
| A15 | ocp_param_base.py:11-23 | docstring example backwards | docs only; report upstream |
| A16 | ROS:554 | debug topic publishes the uncompensated x0 | not ported; report upstream |
| A17 | robot_model.py:282-284 | `coal.Capsule(radius, halfLength)`: coal's second argument is the full length, so each capsule is half as long as the cylinder it replaces (collision avoidance on too-short links). Found in T3.1 | pass `2 * halfLength` (B20); ported as in Python until ruled |
| A18 | ocp_base_croco.py:187-189, mpc.py:76-80 | `integrate` returns `runningDatas[0].xnext`, a view into crocoddyl's data; the state keeps slices of it, so the next solve overwrites the state with node 0's prediction. In ROS (`agimus_controller.py:529-532`, `constant_delay`) the `initial_state` sent with the next control and the feedback's x0 are wrong. Found in T4.1 | `return data.xnext.copy()` upstream; C++ `Mpc::integrate` writes into the `RobotState` (not affected). G10a's generator copies q, v after `integrate` |

Also: ROS `setup_mpc_initial_guess` sets `_first_call = False` without
`clear_past`; covered by the C++ rolling check (B7).

## T2.6 Solver-side time back to `double`: done
- `OcpParams::dt`, `timesteps`, `total_time` and `max_solve_time` are
  `double` seconds. (The note that Python's `sum` adds in order was wrong;
  corrected in T4.1.) Green in configurations A and B (67/67).
- Deviation: `finalize()` also rejects a NaN `dt` and a `max_solve_time`
  that is not > 0 (setup errors; Python does not check).

## T3.1 `RobotModels`: done
- All 24 tests of `test_robot_models.py` ported (8 franka ones skipped
  without `agimus_franka_description` and xacro, as in Python), plus 7 on a
  tiny URDF: capsule conversion, explicit pairs, SRDF XML filtering, free
  flyer, environment attachment, broken URDF. Green in A and B (90/90).
- Rulings:
  - `appendModel` adds to its output model: the collision call gets a fresh
    one (Python gets a new model per call).
  - A capsule replaces the cylinder `<base>_<i>` and the spheres
    `<base>_<i+1>`, `<base>_<i+2>`; the capsule is appended after the
    remaining objects.
- Deviations (setup errors Python lacks): an unknown
  `robot_attachment_frame` throws (Python attaches to `getFrameId`'s
  not-found value); a `q0` not of size `nq` throws. A missing file throws
  `std::runtime_error` (Python: `ValueError`).
- Open: A14 (armature by name) and A17 (capsule length) wait for the user;
  both are ported as in Python.

## T1.3 Project skill `rt-safe-cpp`: done
- `.claude/skills/rt-safe-cpp/SKILL.md`, model-invoked: five steps and the
  red-test triage, pointing to rt_safety.md for every rule. Briefs name it
  in "Rulings in force" for any task with `@rt` code; the first such
  brief (T4.2) checks the done-when (the implementer finds the rules and
  the macros from the skill alone).

## T0.5 Platform matrix: table done, CI proposal pending
- Table, minimums and CI proposal in environment.md, "Platform matrix
  (T0.5)". Supervisor re-checked the ROS versions in rosdistro (pinocchio
  4.1.0, crocoddyl 3.2.1 / 3.2.0 on Lyrical, coal 3.0.3 everywhere).
- Rulings: `find_package(coal)` everywhere (all platforms ship coal 3.0.x);
  minimums are the oldest platform version (crocoddyl 3.2.0, pinocchio
  4.1.0, coal 3.0.3); mim_solvers, colmpc and force_feedback_mpc come from
  source on Ubuntu (not released for any ROS distro); toml++ is missing on
  Humble.
- Open: user approval of the CI proposal; whether ROS's
  `example_robot_descriptions` 7.0.0 installs the same CMake config and
  `share/example-robot-data` layout as nix's 5.0.0 (the T3.1 test locates
  the models through it): the first ROS CI run will tell.

## T4.1 Golden files: done (9372dc0, 1042d71, b9f7287)
- `CPP/test/golden/`: generator, README and fixtures G1–G5, G10a. The
  supervisor reran the generator: byte-identical. 92/92 in A and B; pytest
  58 passed, 8 skipped.
- Rulings:
  - `total_time` reproduces CPython >= 3.12's compensated float `sum`
    (Neumaier), so G1 compares exactly (design.md, Parameters).
  - Fixtures record Python as it is, pending A-rulings: A13 shows in G3
    (`xs[3] == previous_xs[3]`); A4, A6, A12 have no effect on G5/G10a.
  - G10a mirrors the ROS start-up (`setup_mpc_initial_guess`), advances the
    state with `MPC.integrate` (Euler step of node 0, `timesteps[0]`), and
    copies q, v after it (A18). B1/B2 were not needed (`time_ns` stays 0).
  - toml++ is a test-only dependency (`test_ocp_params` links it).
    `package.xml` uses `libtomlplusplus-dev`: rosdep has no key for toml++.
  - `toml-sort-fix` (pre-commit) excludes `test/golden/`: it would reorder
    the generated files.
- Deviations: G2 uses ur5 (its Python test does); G3 uses fixed controls
  instead of random ones; G4 also stores a recomputed solution (max
  difference to the pickle 1.6e-11).
- Open: A18 for the user.
