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
