# agimus_controller_cpp: C++ port plan

Port the Python package `agimus_controller` (this repository,
`agimus_controller/agimus_controller/`) to a real-time-safe C++ library in the
package `agimus_controller_cpp/`. The plan and design documents live in
`agimus_controller_cpp/doc/`. **Speed is a goal of the rewrite:** the MPC tick must be as fast as
possible (ruling R8). A supervisor model drives the work and hands one task
at a time to implementer agents.

Read in this order:

1. This file: rulings, phases and tasks.
2. [doc/supervision.md](supervision.md): how tasks are briefed, executed
   and reviewed. **Implementers read it before their first task.**
3. [doc/design.md](design.md): the C++ API, conventions, the
   Python-to-C++ map and the deliberate behaviour changes. This is the
   rulebook: any translation choice two agents could make differently is
   decided there.
4. [doc/rt_safety.md](rt_safety.md): the real-time rules and the
   allocation-checking test harness.
5. [doc/crocoddyl_yaml.md](crocoddyl_yaml.md): upgrading and extending
   `crocoddyl_yaml`, and the agimus OCP definition files.

Path shorthands used everywhere:

| Shorthand | Path |
|---|---|
| `PY/` | `agimus_controller/agimus_controller/agimus_controller/` (Python sources, at commit `630372d` on `main`) |
| `PYT/` | `agimus_controller/agimus_controller/tests/` (Python tests) |
| `ROS/` | `agimus_controller/agimus_controller_ros/agimus_controller_ros/` |
| `CPP/` | `agimus_controller/agimus_controller_cpp/` (the C++ package; these documents are in `CPP/doc/`) |
| `CY/` | `workspace_cadmium/src/crocoddyl_yaml/` (separate git repo, PAL GitLab) |

## Rulings (decided with the user, 2026-11-24)

Treat these as fixed. Changing one needs the user.

- **R1 Scope.** Port the core: `MPC`, `TrajectoryBuffer` and
  `ConstantTrajectoryBuffer`, the trajectory point types, `interpolate_weights`,
  `OCPParamsBaseCroco`, the debug data, `RobotModels`, `WarmStartReference`,
  `WarmStartShiftPreviousSolution`, and the generic OCP built from YAML. Also port
  **force feedback** (`OCPCrocoForceFeedbackGeneric`, the soft-contact models,
  `WarmStartShiftPreviousSolutionForceFeedback`, the rolling buffer) and the
  hand-written **goal-reaching OCP** (`OCPCrocoGoalReaching`).
  **Not ported:** plots (`PY/plots/`), the empty factory stubs
  (`PY/factory/ocp.py`, `PY/factory/warm_start.py`), the trajectory generators
  (`PY/trajectories/`, used only by the Python publisher node) and Python bindings.
- **R2 OCP definition.** OCPs are described with `crocoddyl_yaml` (`CY/`).
  **The crocoddyl_yaml schema is frozen.** You may add new type files (a new
  residual, activation or contact schema) and implement keys the schema
  already declares. You may not change or remove existing keys. A separate
  translator from the Python YAML format will come later; it is not part of
  this plan. agimus reads two stages of a crocoddyl_yaml trajectory as
  templates ("running" and "terminal"), built once per node. The agimus-only
  keys (update, publish_residual, visual servoing, ...) live in a small agimus
  YAML file. Details: [doc/crocoddyl_yaml.md](crocoddyl_yaml.md).
- **R3 Third-party real-time violations stay red.** If crocoddyl, pinocchio,
  mim_solvers, colmpc, coal or force_feedback_mpc allocates inside the
  real-time path, the test fails and stays failing. The implementer stops and
  reports; the user decides how to patch the dependency.
- **R4 Trajectory points mirror the ROS message.** `WeightedTrajectoryPoint`
  mirrors `agimus_msgs/MpcInput` and its `MpcEEInput[] ee_inputs`, using
  fixed-capacity, pure-C++ containers (no ROS types, no heap in the loop).
- **R5 Simple code.** One concrete `Ocp` class (no `OCPBase` →
  `OCPBaseCroco` hierarchy), one solver (mim_solvers `SolverCSQP`), plain
  structs. A virtual interface exists only where two implementations are
  swapped at runtime (warm starts).
- **R6 Build.** Plain CMake (no ament, no jrl-cmakemodules) with a
  `package.xml` (`build_type` cmake) so colcon builds it. C++17. Each
  translation unit must compile in under 2 GB of RAM (CI limit). Generic
  x86-64 (ISA flags only via T10.7, see R8). Tests use **doctest**, not GTest, because it is much
  lighter. Each test case is registered in CTest with
  `doctest_discover_tests`.
- **R7 Git.** Atomic commits with short messages. No `Co-Authored-By` or
  "Generated with" lines. Never push.
- **R8 Speed, measured.** The time of one `Mpc::run` tick (median, p99, max)
  is the figure of merit. Speed is won in two places:
  - by design, from the start: the hot-path rules in
    [doc/design.md#performance](design.md#performance);
  - by measurement, later: the benchmark exists from P4 on, and P10
    optimizes from profiles.

  This refines R5: start with the simple version, and accept extra
  complexity only when the benchmark shows it pays (rule of thumb: ≥ 3 % on
  the median tick, or a clear p99 gain). Changes inside third-party
  libraries follow the same path as R3: report the numbers and patch only
  with the user's approval.

  Builds target **generic x86-64** by default. An ISA-specific build
  (`-march`, AVX2/FMA, x86-64-v3) may come later, but only through T10.7:
  measure the gain, then get a deployment plan approved by the user. Until
  then, no task adds such flags.

## Phases

Each task has an id, a difficulty (S: a small model can do it; M: a mid
model; H: the supervisor or a strong model), what to read, and a **done-when**
criterion. A task is done only when its done-when holds *and* the supervisor
has reviewed it ([doc/supervision.md](supervision.md)).

```
P0 ─► P1 ─► P2 ─┬─► P3 ─┐
                └─► P4 ─┴─► P7 ─► P8 ─► P9 ─► P10
P0 ─► P5 ─► P6 ───────────► P7
```

P5–P6 (crocoddyl_yaml) can run in parallel with P2–P4.

### P0 Environment and judge (supervisor)

- **T0.1 (H) C++ environment.** In the workspace_cadmium dev container (the
  deployment target; deps live in `/opt/ros/lyrical`), record the version and
  CMake config path of: Eigen, pinocchio, crocoddyl, coal, mim_solvers (built
  in this workspace), colmpc (C++ library, not just Python), force_feedback_mpc
  (C++), example-robot-data, doctest (its CMake config, which provides
  `doctest::doctest` and the `doctest_discover_tests` script), yaml-cpp,
  yaml-schema-cpp (`src/`). Record
  the exact build and test commands (colcon with `--packages-select`).
  **Done when** `doc/environment.md` lists every dependency with a version, or
  marks it MISSING. A MISSING entry is escalated to the user before P1.
- **T0.2 (H) Python judge.** The parity tests compare C++ results with
  "golden" files produced by the Python package. Enter the repo's Nix dev
  shell (`nix develop` at the repo root; `flake.nix` uses gepetto/nix) and run
  `pytest agimus_controller/tests`. Record the Python-side versions of
  crocoddyl, pinocchio, mim_solvers, colmpc and force_feedback_mpc.
  **Done when** the pytest result and both version lists are in
  `doc/environment.md`. If crocoddyl, pinocchio or mim_solvers differ by a
  minor version or more between the two environments, tell the user: parity
  tolerances depend on it.
- **T0.3 (H) Ledger and skills.** Create `doc/ledger.md` (format in
  supervision.md) and install the skills listed in
  [doc/supervision.md#skills](supervision.md#skills).
  **Done when** the ledger exists and the skills load.

### P1 Package skeleton and real-time harness

- **T1.1 (M) Skeleton.** `CPP/CMakeLists.txt`, `CPP/package.xml`, one
  library target `agimus_controller_cpp` (alias
  `agimus_controller_cpp::agimus_controller_cpp`) with a placeholder source,
  install and export of the target and headers (`include/agimus_controller/`),
  `BUILD_TESTING` with doctest, and the `AGIMUS_CONTROLLER_RT_CHECKS` option
  from rt_safety.md. The doctest `main` is compiled once, in the test-support
  library (`DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`), not in every test file. Find
  only the dependencies P1–P4 need: Eigen3, pinocchio, crocoddyl,
  mim_solvers, doctest.
  **Done when** the colcon build from `doc/environment.md` succeeds and
  `ctest` runs (0 tests is fine).
- **T1.2 (M) Allocation harness.** Implement
  [doc/rt_safety.md#harness](rt_safety.md#harness): the test-support
  library, `CHECK_RT_SAFE` / `CHECK_ALLOCATES` / `CHECK_ABORTS`, every
  negative test listed there, and the `check_rt_coverage` ctest.
  **Done when** every negative test passes (each forbidden operation is
  detected); one example `@rt` function has its `Rt*` test and passes; and
  setting `AGIMUS_RT_ABORT_ON_ALLOC=1` makes a negative test abort, which the
  ledger records.
- **T1.3 (H) Project skill.** Write the `rt-safe-cpp` skill (a checklist that
  points to rt_safety.md, the single source of truth) with
  `mattpocock-skills:writing-for-agents`.
  **Done when** an implementer briefed with only the skill name and a task
  finds the RT rules and the harness macros.

### P2 Data types (no crocoddyl)

Read [doc/design.md#data-types](design.md#data-types) first.

- **T2.1 (S) Fixed-capacity containers.** `fixed_capacity.hpp`: `FixedString<N>`
  and `FixedVector<T, N>`. Unit tests, plus `Rt*` tests for every `@rt`
  method, including the full-capacity and too-long-string branches.
- **T2.2 (M) Trajectory points.** `trajectory_point.hpp/.cpp`:
  `EndEffectorInput`, `WeightedTrajectoryPoint`, `RobotState`,
  `make_trajectory_point(nq, nv)`, `find_ee_input`, `interpolate_weights`.
  Sources: `PY/trajectory.py:1-240,425-470`. Python has no test for
  `interpolate_weights`: write new ones, including a frame present in only
  one of the two points (design.md, behaviour change B3).
- **T2.3 (S) OCP parameters.** `ocp_params.hpp/.cpp`: `DtFactorsNSeq`,
  `OcpParams`. Source `PY/ocp_param_base.py`; port
  `PYT/test_ocp_param_base.py`.
- **T2.4 (S) Results and debug data.** `mpc_data.hpp/.cpp`. Source
  `PY/mpc_data.py`; port `PYT/test_mpc_data.py`.
- **T2.5 (M) Trajectory buffer.** `trajectory_buffer.hpp/.cpp`, sliding and
  constant modes. Source `PY/trajectory.py:321-422`; port all 18 tests of
  `PYT/test_buffer.py`, plus `Rt*` tests for `push_back` (including when
  full), `horizon`, `clear_past`, in both modes.

**P2 done when** all of the above build, pass, and `check_rt_coverage` passes.

### P3 Robot models (setup only)

- **T3.1 (M) `RobotModels`.** `robot_models.hpp/.cpp`, port of
  `PY/factory/robot_model.py`: URDF from string or file, environment URDF
  attached with `appendModel`, joint locking with `buildReducedModel`,
  capsule conversion (coal), SRDF self-collision filtering, explicit
  collision pairs. Port `PYT/test_robot_models.py`; tests that need
  `agimus_franka_description` carry the decorator
  `* doctest::skip(!franka_description_found())`, so they show up as skipped
  when it is not installed.
  **Done when** the ported tests pass on example-robot-data's panda.

### P4 OCP core, warm starts and MPC (hand-built models)

Read [doc/design.md#ocp](design.md#ocp) and the warm-start and MPC sections.

- **T4.1 (H) Golden files.** `CPP/test/golden/generate_golden.py` (runs in
  the Python judge environment from T0.2) writes YAML fixtures for golden
  sets G1–G5 and G10a ([doc/design.md#golden-files](design.md#golden-files)),
  plus `CPP/test/golden/README.md` on how to regenerate them.
  **Done when** the fixtures are committed and regenerating them reproduces
  identical files.
- **T4.2 (H) `Ocp` and cost updaters.** `ocp.hpp/.cpp` and
  `cost_updater.hpp/.cpp`: problem and CSQP construction, `solve`,
  `integrate`, results, debug data, geometry placement, and the updater rules
  for state, control, control_grav and the frame
  placement/translation/rotation/velocity residuals (fixed frame and frame
  from the trajectory). Sources: `PY/ocp_base_croco.py`,
  `PY/ocp/ocp_croco_generic.py:153-433,713-892`. Port
  `PYT/test_ocp_croco_base.py` (its `.pkl` becomes golden set G4).
- **T4.3 (M) Goal-reaching builder.** `build_goal_reaching_models` in
  `ocp_builders.hpp/.cpp`, port of `PY/ocp/ocp_croco_goal_reaching.py`. Port
  `PYT/test_ocp_croco_goal_reaching.py` against golden set G5.
- **T4.4 (M) Warm starts.** `warm_start.hpp/.cpp`: the `WarmStart` interface,
  `WarmStartReference`, `WarmStartShiftPreviousSolution`. Sources
  `PY/warm_start_*.py` (except force feedback). Port
  `PYT/test_warm_start_reference.py` and
  `PYT/test_warm_start_shift_previous_reference.py` against golden sets G2 and
  G3.
- **T4.5 (H) `Mpc`.** `mpc.hpp/.cpp`, source `PY/mpc.py`. Port
  `PYT/test_mpc_unicycle.py` using crocoddyl's `ActionModelUnicycle` through
  `Ocp`. Add the closed-loop golden test G10a: goal-reaching OCP, 20 ticks,
  with the state advanced by `Mpc::integrate`. Add `RtMpc.run`: warm up, then
  100 ticks under `CHECK_RT_SAFE`, with `n_threads = 1`.
  **This is where third-party allocations show up (R3).** If the test goes
  red inside crocoddyl or mim_solvers code, stop and follow
  [doc/rt_safety.md#third-party-violations](rt_safety.md#third-party-violations).
- **T4.6 (M) Threads.** Run `RtMpc.run` again with `n_threads = 4` (OpenMP) as
  a separate test, and record the result in the ledger. A red result is
  escalated (R3).
- **T4.7 (M) Benchmark.** `CPP/benchmark/bench_mpc.cpp` follows
  [doc/design.md#benchmark](design.md#benchmark):
  - goal-reaching OCP: per-tick time split into references, warm start,
    solve and bookkeeping;
  - the same numbers for a bare-crocoddyl baseline (the same problem built
    and solved directly, without the agimus layer);
  - the Python `MPC.run` on the same problem (from the T0.2 environment).

  **Done when** the numbers are in the ledger as the first baseline.

**P4 done when** all tests are green, or the only red ones are third-party
RT failures that have been reported to the user, and the T4.7 baseline is
recorded.

### P5 crocoddyl_yaml upgrade (repo `CY/`)

Read [doc/crocoddyl_yaml.md](crocoddyl_yaml.md) completely before the first task.

- **T5.1 (M) Build as-is.** Add a `package.xml`, install the schemas, and
  build in the container. **Done when** the compile errors are listed in the
  ledger, or the build passes.
- **T5.2 (M) API migration.** Switch from `boost::shared_ptr` to `std::shared_ptr`
  and fix the other crocoddyl/pinocchio API breaks (changelog walk described
  in crocoddyl_yaml.md). **Done when** the library and the 7 registered tests
  build and pass.
- **T5.3 (M) Defects.** Fix the defects listed in
  [doc/crocoddyl_yaml.md#defects](crocoddyl_yaml.md#defects), one commit and one test per fix.
- **T5.4 (M) Build context.** Add the factory API that takes a caller-supplied
  state, actuation and geometry model
  ([doc/crocoddyl_yaml.md#api-additions](crocoddyl_yaml.md#api-additions)).
  Keep the existing overloads.

### P6 crocoddyl_yaml extensions

Each new type is a new schema file, a factory entry, a test YAML and a
test case. The new tests go in crocoddyl_yaml's existing gtest suite: that
repo is not migrated to doctest (doctest is the rule for `CPP/` only). See [doc/crocoddyl_yaml.md#new-types](crocoddyl_yaml.md#new-types).

- **T6.1 (S)** Residual `control_grav`.
- **T6.2 (M)** Activations `colmpc_exp`, `colmpc_quad_exp`, behind the
  `CROCODDYL_YAML_WITH_COLMPC` CMake option.
- **T6.3 (H)** Residuals `colmpc_distance_collision`,
  `colmpc_distance_collision2`, collision-pair registration, and automatic
  selection of the colmpc state and DAM.
- **T6.4 (M)** Constraints: implement the `constraints` key that
  `stage.schema` already declares. If `constraint.schema` cannot validate
  without an edit, stop and ask (R2).

### P7 OCP from YAML

- **T7.1 (M) agimus OCP file.** `CPP/schemas/agimus_ocp.schema`
  (yaml-schema-cpp) and `ocp_definition.hpp/.cpp`, which load the file into
  an `OcpDefinition` struct. The format is in
  [doc/crocoddyl_yaml.md#the-agimus-ocp-file](crocoddyl_yaml.md#the-agimus-ocp-file).
- **T7.2 (H) Stage-template builder.** `build_models_from_yaml` in
  `ocp_builders.cpp`: builds one model per node from the running stage with
  per-node dt, a terminal model with dt 0, armature, inactive costs, updaters
  matched by cost name and residual type, the published residual list,
  visual-servoing input transforms, and collision weights.
- **T7.3 (M) Test OCP files.** Hand-convert to crocoddyl_yaml plus agimus
  files under `CPP/test/resources/ocp/`:
  - `PY/ocp/ocp_goal_reaching.yaml`
  - the panda example in `agimus_controller_examples`
  - demo 08's collision OCP (quoted in crocoddyl_yaml.md)
  - a visual-servoing OCP modelled on demo 04

  Write the mapping decisions to `CPP/test/resources/ocp/CONVERSION.md`; it
  becomes the input spec for the future translator.
- **T7.4 (M) YAML parity.** Golden sets G6–G8 against Python
  `OCPCrocoGeneric` on the same problems, plus `Rt*` tests of
  `set_reference_weighted_trajectory` and `solve` with collision and visual
  servoing.
- **T7.5 (M) Closed loop on YAML.** Golden set G10b: the G10a loop on the
  YAML goal-reaching OCP.

### P8 Force feedback

- **T8.1 (H) CY soft contacts.** Wire the stage's `contacts` key into the DAM
  factory and add the contact types `soft_1d` and `soft_3d`
  (force_feedback_mpc `DAMSoftContact{1D,3D}AugmentedFwdDynamics`).
  `stage.schema` declares `contact[]`, but no `contact.schema` exists. Expect a
  stop for a schema decision (crocoddyl_yaml.md, section on declared keys).
- **T8.2 (H) agimus side.** Build the `IAMSoftContactAugmented` per node (the
  `soft_contact` section of the agimus file), add the soft-contact updater
  (f_des, f_weight, active_contact, tau_grav_weight) and the update of the
  IAM bounds, and handle the state size nx + nc. Source
  `PY/ocp/ocp_croco_generic_force_feedback.py`.
- **T8.3 (M) Force-feedback warm start and rolling buffer.**
  `WarmStartShiftPreviousSolutionForceFeedback`, and the
  `expect_rolling_buffer` mode (`ShootingProblem::circularAppend`). Source
  `PY/warm_start_shift_previous_solution_force_feedback.py`.
- **T8.4 (M) Parity.** Golden set G9 plus `Rt*` tests.

### P9 Hardening and handover

- **T9.1 (S) RT coverage audit.** `check_rt_coverage` passes, and the
  supervisor spot-checks that every `@rt` test exercises the realistic branch.
- **T9.2 (M) Optional RTSan job.** If clang 20 or newer is available, add a
  `-fsanitize=realtime` build of the `Rt*` tests (see rt_safety.md).
- **T9.3 (S) Benchmark coverage.** Extend `bench_mpc` to the YAML
  goal-reaching, collision, visual-servoing and force-feedback OCPs, and
  record them in the ledger.
- **T9.4 (S) README.** API overview, build, RT rules (link), YAML files, the
  list of behaviour changes from design.md, and the benchmark table.
- **T9.5 (H) Final review.** A whole-branch review with the `code-review`
  skill. Every finding is fixed or recorded in the ledger as accepted.

### P10 Optimization

Every task here starts from a profile and ends with before/after benchmark
numbers in the ledger. A change that does not meet R8's bar is reverted.
Golden and RT tests stay green throughout.

- **T10.1 (H) Profile.** Record `perf` profiles of `bench_mpc` (Release
  build, `n_threads = 1`) for each OCP. Report where a tick goes:
  - our code: updaters, warm start, copies, debug data;
  - crocoddyl `calc` / `calcDiff`, split by model component (dynamics,
    costs, collision distance);
  - the CSQP QP iterations;
  - the line search.

  **Done when** the ledger has the ranked table, and the supervisor has
  shown it to the user before T10.4–T10.6 start.
- **T10.2 (M) agimus overhead.** Bring `Mpc::run` minus the solver's
  `solve` to ≤ 2 % of the tick on every OCP (measured against the T4.7
  bare-crocoddyl baseline). Typical work: remove copies, skip debug-data
  work that is disabled, and cache what the updaters recompute.
- **T10.3 (M) Build settings.** Measure `-O2` against `-O3`, and LTO on our
  library only, on generic x86-64. Keep a flag only if it pays (R8) and every
  translation unit still compiles under 2 GB.
- **T10.4 (H) Threads.** Measure the tick with `n_threads` 1, 2, 4 and the
  number of cores, setting the thread count explicitly (some builds default
  to many OpenMP threads; also set `OMP_THREAD_LIMIT`). Recommend a default
  per OCP size.
- **T10.5 (H) Hot spots in dependencies.** For each third-party hot spot
  from T10.1 (e.g. collision distance, frame kinematics, the QP), give the
  user the numbers and a proposed patch. Implement only the approved ones,
  in their own repos.
- **T10.6 (H) Alternative solver (needs the user's go-ahead).** `Ocp` holds
  its solver through `crocoddyl::SolverAbstract` (design.md), so another
  solver fits behind the same API. The user's `SolverStagewiseSQP` (ariadne,
  branch `stagewise-solver`) is the candidate. Compare its solutions with
  CSQP's on every golden problem, then benchmark.
- **T10.7 (H) ISA-specific build: gain and deployment plan (needs the user's
  approval before anything ships).**
  1. **Measure.** Build the whole stack (Eigen users: pinocchio, crocoddyl,
     coal, colmpc, mim_solvers, crocoddyl_yaml and this package) at
     `-march=x86-64-v2`, `x86-64-v3` and `native`. Compare `bench_mpc`
     with the generic build. Use the portable levels v2/v3 rather than
     `native`, because they name a fixed instruction set a robot CPU
     either has or lacks. `native` is only an upper bound.
  2. **Deployment plan** (`doc/isa_deployment.md`), written only if the
     gain meets R8. It must answer:
     - which CPUs every target robot and CI machine has, and which levels
       they support (`ld.so --help` lists the supported glibc-hwcaps
       levels);
     - how one stack stays consistent. Every library sharing Eigen
       objects must use the same `-march` and `EIGEN_MAX_ALIGN_BYTES`.
       Mixing AVX-built and generic-built Eigen code in one process breaks
       alignment assumptions and crashes, as seen with moto's aligned
       maps. So the unit of deployment is the whole stack, not one
       library;
     - how the right build is selected. Options: ship generic plus v3
       builds and let the glibc dynamic loader pick from
       `glibc-hwcaps/x86-64-v3/` (glibc ≥ 2.33), or ship separate packages
       per robot, or check the CPU at startup and refuse a mismatched build
       with a clear error, never a SIGILL;
     - how CI builds and tests every shipped variant, including the golden
       and RT tests, and how a new robot CPU is handled;
     - rollback to the generic build.
  3. **Done when** the numbers are in the ledger and, if the gain meets
     R8, the user has approved or rejected `doc/isa_deployment.md`.

## Facts the plan relies on

- The real-time path of the stock ROS node (`ROS/agimus_controller.py:483-556`)
  per tick: optional `Mpc::integrate` (constant delay), `Mpc::run`, then the
  first node's feedforward and Riccati gain go into the control message.
  `WarmStartReference` is used once, for the initial solve with no iteration
  limit (`ROS/agimus_controller.py:368-390`).
- Users in agimus-demos (commit `5586a0e`):
  - Every demo except 07_deburring uses the stock node with
    `OCPCrocoGeneric` and a YAML file.
  - 07_deburring uses force feedback from Python, through `agimus_pytroller`.
  - Demos 07 and 08 import `interpolate_weights`.
  - agimus-demos pins agimus_controller `c62ed3b` (`humble-devel`), which is
    not an ancestor of the `main` commit being ported (`630372d`).
- crocoddyl_yaml was written between 2024-12-16 and 2024-12-20, against
  crocoddyl 2.x (`boost::shared_ptr` API) and pinocchio 3.x.
