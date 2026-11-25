# Real-time safety

The MPC runs inside a control loop. A function tagged `@rt` must take a
bounded time and never block, so it does not allocate, free, throw, lock, log
or do I/O. A test proves this for every `@rt` function. **Code in the RT path
that is not RT-safe must make a test go red.**

## Rules for `@rt` code

1. Size every buffer at setup: Eigen vectors and matrices, `std::vector`s of
   them, `FixedVector`s, solver workspaces. RT code writes only into storage
   that already has the right size.
2. Assign Eigen objects of equal size (`dst = src`, `dst.noalias() = A * x`).
   An expression that creates a temporary of dynamic size allocates. Write
   it into a preallocated member instead. `pinocchio::SE3`, `Motion`,
   `Force` and fixed-size Eigen types never allocate.
3. Containers: no `push_back`, `resize`, `insert` or `reserve` on standard
   containers; no `std::string` construction or concatenation; no `std::map`
   or `std::unordered_map`. Use `FixedVector`, `FixedString` and
   index-addressed `std::vector`s sized at setup.
4. No `throw`, and no call that can throw on valid input. Return a `Status`.
5. No `new`, `make_shared`, `shared_ptr` copies, `std::function` holding a
   large capture, `dynamic_cast` or `dynamic_pointer_cast`. Resolve types at
   setup and cache raw pointers.
6. No logging, no `std::cout`, no file access, no mutex. A solver callback
   that prints is a setup or debug feature and is documented as non-RT.
7. Warm-up is setup. Some libraries allocate lazily on their first call
   (first `calc`, the OpenMP thread pool). The setup phase therefore ends
   with one full call of the RT path. The ROS node does the same: an initial
   solve before the loop starts.
8. Threads: the RT tests use `n_threads = 1`. The `n_threads > 1` test
   (T4.6) is separate.

## Harness

Lives in `CPP/test/rt/`. The test-support static library is
`agimus_rt_test_support`. Every test executable links it, and it also holds
the single doctest `main` (`DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`).

### Allocation hook (`allocation_hook.cpp`)

Replaces, in the test executable:

- every global `operator new` and `operator delete`: plain, array, nothrow,
  `std::align_val_t`, and sized variants;
- `malloc`, `free`, `calloc`, `realloc`, `posix_memalign`, `aligned_alloc`,
  `memalign`, `valloc`, `pvalloc`. These forward to glibc's `__libc_malloc`,
  `__libc_free`, `__libc_calloc`, `__libc_realloc` and `__libc_memalign`
  (declared `extern "C"`).

Because the executable defines these symbols, the dynamic linker resolves
calls from shared libraries (crocoddyl, pinocchio, coal, mim_solvers) to
them. That is how allocations inside third-party code are seen. Eigen's
`aligned_malloc` calls `malloc`, so it is seen too.

- A `thread_local` flag turns counting on. A `thread_local` counter records
  the number of calls and the bytes requested. Only allocating calls
  (`malloc`, `new`, `calloc`, `realloc`, the aligned variants) count. A
  `free` or `delete` counts too, because freeing in the loop is just as
  forbidden.
- With the environment variable `AGIMUS_RT_ABORT_ON_ALLOC=1`, the first
  counted call calls `std::abort()`. Running the failing test under gdb then
  gives the backtrace of the offender.
- The hook functions never allocate themselves. Read the environment
  variable once, in a static initialiser.
- Do not combine the hook with ASan or another allocator replacement. Give
  sanitizer builds their own configuration without the hook.

### Guard and macros (`rt_guard.hpp`)

- `class ScopedRtGuard`: the constructor resets the counters, sets the
  thread-local flag and calls `Eigen::internal::set_is_malloc_allowed(false)`.
  The destructor restores both. `allocations()` and `bytes()` read the
  counters.
- `CHECK_RT_SAFE(statement)` runs `statement` inside a guard, then does
  `CHECK_MESSAGE(guard.allocations() == 0, ...)` with a message naming the
  statement, the count and the bytes.
- `CHECK_ALLOCATES(statement)` is the opposite. It is used by the negative
  tests.
- `CHECK_ABORTS(statement)` stands in for death tests, which doctest does
  not have.
  - It `fork()`s; the child runs `statement` and calls `_exit(0)`.
  - The parent `waitpid`s and checks that the child was killed by `SIGABRT`.
  - The statement runs in a capture-less lambda, so it cannot use the
    test's local variables.
  - Use it only in the negative-test executable, which stays single-threaded
    (no OpenMP), so forking is safe.
- An Eigen allocation under the guard, in code compiled with
  `EIGEN_RUNTIME_NO_MALLOC`, **aborts** the test executable rather than
  being counted, because Eigen's check fires before the allocation reaches
  the hook. doctest reports the crash with the test name. Other allocations
  are counted and reported by `CHECK_RT_SAFE`.
- Never put doctest assertions inside a guarded statement, because doctest
  allocates.
- The compiler may elide a `new`/`delete` pair whose pointer does not
  escape, so negative tests write their results and pointers to `volatile`
  globals.

### Eigen flags

CMake option `AGIMUS_CONTROLLER_RT_CHECKS`, default `ON` when `BUILD_TESTING`
is on. When ON, it adds to the library target (PUBLIC, so the tests get the
same definitions and every translation unit agrees):

- `EIGEN_RUNTIME_NO_MALLOC`, which makes `set_is_malloc_allowed(false)`
  effective: Eigen asserts on any heap allocation in code compiled in our
  translation units;
- `-UNDEBUG`, because Eigen's checks are `eigen_assert`s and need
  assertions on. RT-check builds are therefore not release builds.
  Production and benchmark builds set the option OFF (design.md,
  Performance section).

The allocation hook is the source of truth: it also sees allocations in
precompiled third-party code, which Eigen's flag cannot see. The Eigen flag
adds an exact abort location for our own code.

`EIGEN_NO_AUTOMATIC_RESIZING` is **not** used. Eigen 3.4 checks it only in
`PlainObjectBase::_resize_to_match`; plain assignment goes through
`resize_if_allowed` (`Eigen/src/Core/AssignEvaluator.h`) and resizes
silently. A size-changing assignment under the guard is still caught,
because the resize reallocates and `EIGEN_RUNTIME_NO_MALLOC` aborts.

### Negative tests (`test_rt_harness.cpp`)

These prove the harness goes red. Each uses `CHECK_ALLOCATES`, or
`CHECK_ABORTS` for the Eigen asserts:

- `std::vector<double>` growth (`push_back` past capacity);
- a `std::string` longer than the small-string buffer;
- `std::make_shared<int>()`;
- `throw` plus `catch` of a `std::runtime_error`;
- a `delete` of memory allocated before the guard;
- a C `malloc`;
- an allocation inside our shared library (`compute_horizon_indexes`),
  which proves that calls from shared libraries are interposed;
- `CHECK_ABORTS` of `Eigen::VectorXd` construction with a dynamic size under
  the guard (proves `EIGEN_RUNTIME_NO_MALLOC` is active);
- `CHECK_ABORTS` of a dynamic-size temporary, `Eigen::VectorXd y = A * x;`;
- `CHECK_ABORTS` of a resizing assignment `a = b` (sizes 3 and 4);
- a control: the same Eigen allocation without the guard does not abort, so
  `CHECK_ABORTS` cannot pass vacuously;
- positive controls: fixed-size Eigen work, and a dynamic `A * x` into
  preallocated storage, are RT-safe.

### Coverage rule (`check_rt_coverage`)

- Every header tag `/// @rt Type::method` needs at least one doctest test
  case whose name starts with `RtType.method`, e.g.
  `TEST_CASE("RtMpc.run: 100 ticks after warm-up")`. `TEST_CASE_FIXTURE`
  follows the same naming rule.
- A free function `/// @rt name` needs a test case named `RtFree.name...`.
- Each such test exercises the method under `CHECK_RT_SAFE` with realistic
  sizes, after warm-up, and covers every early-return branch (full buffer,
  unknown frame, missing transform, ...).

`CPP/test/check_rt_coverage.py` greps for the tags and test names and fails
listing the uncovered ones. It is registered as a ctest, so a new `@rt`
function without a test turns the build red.

## Third-party violations

Ruling R3: a third-party allocation in the RT path stays red. When an `Rt*`
test fails and the allocation is not in `CPP/` code:

1. Rerun the single test with `AGIMUS_RT_ABORT_ON_ALLOC=1` under gdb and save
   the backtrace.
2. Find the first frame outside `CPP/` and record the library, the function
   and the trigger (first call only? every call? a size change?).
3. If it allocates only on the first call, check that warm-up covers it. If
   the test still fails after a correct warm-up, it is a real violation.
4. Stop. Report to the supervisor: the test name, the backtrace and a
   proposed patch site. The supervisor escalates to the user. Do not wrap,
   skip or disable the test.

## Optional: RealtimeSanitizer (T9.2)

Clang 20 and newer provide `-fsanitize=realtime`. Functions marked
`[[clang::nonblocking]]` are checked at run time for allocations, locks and
blocking system calls, including inside libraries. If the container has
clang 20 or newer, add a separate build of the `Rt*` tests with RTSan. Mark
the test lambdas, not the library API, so the library keeps building with
GCC. This catches locks and system calls, which the allocation hook cannot
see.
