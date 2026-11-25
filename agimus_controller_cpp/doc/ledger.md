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
