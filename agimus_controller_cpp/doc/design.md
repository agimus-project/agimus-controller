# Design rulebook

The single source of truth for how Python constructs become C++. When this
file and your intuition disagree, follow this file. When this file is silent
or ambiguous, stop and ask the supervisor; do not guess (see supervision.md).

Shorthands (`PY/`, `PYT/`, `ROS/`, `CPP/`, `CT/`) are defined in
[PLAN.md](PLAN.md).

## Conventions

- C++17. Namespace `agimus_controller`. Headers in
  `CPP/include/agimus_controller/`, sources in `CPP/src/`, tests in `CPP/test/`.
- Formatting: clang-format with `--style=Google` (repo pre-commit); CMake
  formatted with gersemi. Run `pre-commit run --files <changed files>` before
  committing.
- Naming follows crocoddyl: `CamelCase` types, `snake_case` functions and
  methods, `snake_case_` private members, `kCamelCase` constants. File names
  are `snake_case.hpp/.cpp`, one main type per file.
- **Two phases.** Every class has a setup phase and a real-time (RT) phase.
  - *Setup* means constructors and anything that takes names, paths or TOML,
    loads files, or sizes buffers. Setup may allocate. On bad input it throws
    `std::invalid_argument` (bad argument) or `std::runtime_error`
    (environment, e.g. a missing file), with a message that names the
    offending item.
  - *RT* functions carry a tag naming them on the line before their
    declaration in the header, e.g. `/// @rt Mpc::run` or
    `/// @rt interpolate_weights`. They follow [rt_safety.md](rt_safety.md):
    no allocation, no throw, no I/O, no locks. They report failure through
    `Status`. In this document, `/// @rt` in front of a signature is
    shorthand for that tag.
- `enum class Status { kOk, kNotEnoughData, kBufferFull, kUnknownFrame,
  kMissingTransform, kSizeMismatch, kNoPreviousSolution };` lives in
  `status.hpp`, with `const char* to_string(Status)`. Add a value only through
  the supervisor.
- A solver that does not converge is not an error: `Ocp::solve` returns the
  CSQP boolean and stores it in the debug data.
- Ownership: `std::shared_ptr` where crocoddyl or pinocchio require it, at
  setup. RT code reaches objects through raw non-owning pointers cached at
  setup. It never calls `dynamic_pointer_cast` or copies a `shared_ptr`.
- Python `assert` becomes:
  - in setup, a throw;
  - in RT, a `Status`, or `assert()` for internal invariants that only a bug
    can break.
- Python `None` ("not provided") has no C++ equivalent. Every vector exists
  and is sized at setup. An absent end effector is one that is not in
  `ee_inputs`. An optional setting (`max_solve_time`) is `std::optional`.
- **Time: `std::chrono` outside, `double` in the solver (PR #334).**
  - agimus' external API uses `std::chrono::nanoseconds`: time stamps of
    trajectory points and of the measured state, the time given to and
    returned by `Mpc`, and every measured duration (debug timers,
    benchmark).
  - The solver side uses `double` seconds, as crocoddyl, CSQP and the
    Python package do: `OcpParams::dt`, `timesteps`, `total_time` and
    `max_solve_time`, crocoddyl models' `dt`, and crocoddyl-toml.
  - Convert only where the two meet, with
    `std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds))`
    one way and `std::chrono::duration<double>(d).count()` the other.
  - Clock: `std::chrono::steady_clock`, which is monotonic and has
    nanosecond resolution on Linux (`CLOCK_MONOTONIC`). Do not use
    `high_resolution_clock`: libstdc++ aliases it to `system_clock`, which
    jumps when the wall clock is adjusted.
  - Read the clock with `steady_clock::now()`, which does not allocate or
    block; it is allowed in `@rt` code.

## Python to C++ map

| Python | C++ | File |
|---|---|---|
| `TrajectoryPoint`, `TrajectoryPointWeights`, `WeightedTrajectoryPoint` | `WeightedTrajectoryPoint` (mirrors `MpcInput`), `EndEffectorInput` (mirrors `MpcEEInput`), `RobotState` | `trajectory_point.hpp` |
| `interpolate_weights` | `interpolate_weights` | `trajectory_point.hpp` |
| `TrajectoryBuffer`, `ConstantTrajectoryBuffer` | `TrajectoryBuffer` with `BufferMode::kSliding` / `kConstant` | `trajectory_buffer.hpp` |
| `DTFactorsNSeq`, `OCPParamsBaseCroco` | `DtFactorsNSeq`, `OcpParams` | `ocp_params.hpp` |
| `OCPResults`, `OCPDebugData`, `MPCDebugData` | `OcpResults`, `OcpDebugData`, `MpcDebugData` | `mpc_data.hpp` |
| `RobotModelParameters`, `RobotModels` | same names | `robot_models.hpp` |
| `OCPBase`, `OCPBaseCroco`, `OCPCrocoGeneric`, `OCPCrocoForceFeedbackGeneric` | one class `Ocp`, fed by an `OcpModels` struct from a builder | `ocp.hpp` |
| `OCPCrocoGoalReaching.create_*` | `build_goal_reaching_models` | `ocp_builders.hpp` |
| YAML dataclasses in `ocp_croco_generic.py` (`build`) | crocoddyl-toml (`CT/`) + `build_models_from_toml` | `ocp_builders.hpp` |
| the same dataclasses (`update`) | `CostUpdater`, `NodeUpdater` | `cost_updater.hpp` |
| `WarmStartBase` and its 3 subclasses | `WarmStart` (abstract) and 3 subclasses | `warm_start.hpp` |
| `MPC` | `Mpc` | `mpc.hpp` |

## Data types

### Fixed-capacity containers (`fixed_capacity.hpp`)

- `FixedString<N>`: `std::array<char, N + 1>` plus a length.
  - Setup constructor from `std::string_view` throws if too long.
  - `/// @rt bool assign(std::string_view)` returns false if too long.
  - `/// @rt` equality with `FixedString` and `std::string_view`.
  - `/// @rt std::string_view view()`.
- `FixedVector<T, N>`: `std::array<T, N>` plus a size.
  - `/// @rt` methods: `push_back` (returns false when full), `clear`,
    `size`, `operator[]`, `begin`/`end`.
  - Elements are constructed at setup, so a `T` holding an Eigen dynamic
    vector keeps its storage across `clear`.
- Constants in `trajectory_point.hpp`: `kMaxEndEffectors = 8` and
  `kMaxFrameNameLength = 63`. `using FrameName = FixedString<kMaxFrameNameLength>;`.

### Trajectory points (`trajectory_point.hpp`)

These structs mirror the ROS messages field for field, so the future ROS
node converts messages with plain copies.

```cpp
struct EndEffectorInput {            // agimus_msgs/MpcEEInput
  FrameName frame_name;              // frame_id
  pinocchio::SE3 pose;     Vector6d w_pose;
  pinocchio::Motion twist; Vector6d w_twist;
  pinocchio::Force force;  Vector6d w_force;
};

struct WeightedTrajectoryPoint {     // agimus_msgs/MpcInput
  int id = -1;
  std::chrono::nanoseconds time{0};
  Eigen::VectorXd q;                 // size nq
  Eigen::VectorXd w_q;               // size nv: the state residual lives in the tangent space
  Eigen::VectorXd qdot, w_qdot;      // size nv
  Eigen::VectorXd qddot, w_qddot;    // size nv
  Eigen::VectorXd robot_effort, w_robot_effort;  // size nv
  double w_collision_avoidance = 0.0;
  FixedVector<EndEffectorInput, kMaxEndEffectors> ee_inputs;
};

struct RobotState {                  // measured state given to Mpc::run
  std::chrono::nanoseconds time{0};
  Eigen::VectorXd q, v, a;           // a defaults to zero, as in the ROS node
  Eigen::Vector3d contact_force = Eigen::Vector3d::Zero();  // force feedback only
};
```

- `make_trajectory_point(nq, nv)` and `make_robot_state(nq, nv)` size every
  vector (weights at zero). `ee_inputs` starts empty.
- `/// @rt const EndEffectorInput* find_ee_input(const WeightedTrajectoryPoint&, std::string_view name)`
  scans linearly and returns nullptr if the name is absent.
- `/// @rt bool same_sizes(const WeightedTrajectoryPoint&, const WeightedTrajectoryPoint&)`
  is true when every vector of both points has the same size; the buffer
  uses it to refuse a point that would reallocate a slot.
- `operator==` compares every field exactly (different sizes compare
  unequal); it is used by tests.
- `/// @rt Status interpolate_weights(const WeightedTrajectoryPoint& a, const WeightedTrajectoryPoint& b, double alpha, WeightedTrajectoryPoint& out)`
  writes only the `w_*` fields and the `ee_inputs` weights of `out`. `alpha`
  is clamped to [0, 1]. The end effectors are the union by name; a frame
  missing on one side counts as zero weights there (behaviour change B3).
  - A union frame missing from `out` is appended with identity pose.
  - A frame of `out` that is in neither input gets zero weights.
  - It returns `kSizeMismatch` if the weight vectors differ in size, and
    `kBufferFull` if the union exceeds `kMaxEndEffectors`.
- Python splits poses, velocities and forces into three dicts keyed by frame.
  Here they sit in one `EndEffectorInput` per frame, as in the message.
  Python's `w_end_effector_poses[f]` is `find_ee_input(p, f)->w_pose`, and so on.

### Buffer (`trajectory_buffer.hpp`)

`TrajectoryBuffer(const DtFactorsNSeq&, std::size_t capacity, BufferMode, const WeightedTrajectoryPoint& prototype)`

- Setup copies `prototype` into every slot, so slots are sized. Storage is a
  ring of `capacity` slots. In sliding mode `capacity` must exceed
  `horizon_indexes().back()` (setup throws otherwise); constant mode ignores
  it.
- `compute_horizon_indexes(const DtFactorsNSeq&)` (setup) gives the node
  indexes: 0, then the cumulative factors (Python
  `TrajectoryBuffer.compute_horizon_indexes`).
- `/// @rt Status push_back(const WeightedTrajectoryPoint&)`:
  - `kSizeMismatch` if the point is not sized like the prototype;
  - sliding mode: copy into the next slot, `kBufferFull` when full;
  - constant mode: overwrite the single latest point.
- `/// @rt void clear_past()`: sliding mode drops the oldest point; constant
  mode does nothing.
- `/// @rt bool has_horizon() const`: sliding mode `size() > horizon_indexes().back()`,
  constant mode "one point received".
- `/// @rt const std::vector<const WeightedTrajectoryPoint*>& horizon()`:
  fills a vector of `n_controls + 1` pointers preallocated at setup.
  Precondition: `has_horizon()`. Constant mode returns the same pointer
  `n_controls + 1` times.
- `size()`, `operator[]`, `back()` and `horizon_indexes()` behave as in
  Python, including constant mode's `size() == 3 * (horizon_indexes().back() + 1)`
  once a point has arrived (see `PY/trajectory.py:270-276`; the ROS node
  depends on it).

### Parameters and results

- `OcpParams` holds the fields of `OCPParamsBaseCroco` (`PY/ocp_param_base.py:25-85`)
  with the same defaults, plus `use_filter_line_search` as a normal field
  (behaviour change B8). `dt`, `timesteps` and `total_time` are `double`
  seconds, and `max_solve_time` is `std::optional<double>`, as in Python
  (solver side; see Conventions, Time).
  `void finalize()` computes `n_controls`, `timesteps` and `total_time`, and
  throws if `horizon_size != n_controls`. `total_time` is summed as
  CPython >= 3.12's `sum()` does (Neumaier compensation), so it equals the
  judge's value exactly; Python 3.10 (Humble) sums left to right and may
  differ by an ulp.
- `OcpResults`: `std::vector<Eigen::VectorXd> states, feed_forward_terms;
  std::vector<Eigen::MatrixXd> riccati_gains;`. Python's typo `ricatti` is
  fixed. `resize(n_controls, nx, ndx, nu)` sizes it at setup, and
  `/// @rt Status copy_results(const OcpResults& src, OcpResults& dst)` copies
  element by element (`kSizeMismatch`, `dst` unchanged, if the shapes
  differ). Warm starts use it in `update_previous_solution`.
- `OcpDebugData`: `result`, `references` and `residuals` as
  `std::vector<NamedMatrix>` (`struct NamedMatrix { std::string name; Eigen::MatrixXd data; }`,
  names and sizes set at setup), plus `kkt_norm`, `nb_iter`, `nb_qp_iter` and
  `problem_solved`.
  - A reference is stored as the ROS node publishes it: a vector is n×1; an
    SE3 is 7×1 XYZQUAT (`pinocchio::SE3ToXYZQUAT`); a rotation is 3×3.
  - Residuals are `n_controls × nr`.
- `MpcDebugData`: `ocp`, `reference_id`, and the four `duration_*` timers
  (`std::chrono::nanoseconds`), measured with `std::chrono::steady_clock`.

## Robot models

`RobotModels` is setup-only and ports `PY/factory/robot_model.py`. Python
accepts a `Path` or an XML `str` for the URDF, environment URDF and SRDF. In
C++ each becomes two `std::string` fields, `*_xml` and `*_path`; at most one
is set, and setting both throws. Validation is that of `__post_init__`.
Accessors: `full_robot_model()`, `robot_model()`, `collision_model()`,
`visual_model()`, `armature()`, `params()`. The models are owned by value and
handed out as const references; the shared pointers that crocoddyl needs are
created by the OCP builders.

## OCP

### `OcpModels`: what a builder produces

```cpp
struct OcpModels {
  std::shared_ptr<crocoddyl::StateAbstract> state;    // crocoddyl or colmpc StateMultibody; any state in hand-built tests (unicycle)
  std::shared_ptr<pinocchio::GeometryModel> geometry; // the object the collision residuals use; nullptr without collisions
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running;  // n_controls, one object per node
  std::shared_ptr<crocoddyl::ActionModelAbstract> terminal;
  std::vector<NodeUpdater> running_updaters;          // one per running node
  NodeUpdater terminal_updater;
  std::vector<std::string> reference_costs;           // debug: costs with update rules (not collision)
  std::vector<std::string> residual_costs;            // debug: publish_residual
  std::vector<TransformInput> input_transforms;       // visual servoing: (parent, child) frame names
  std::size_t force_dim = 0;                          // force feedback: nc, else 0
  bool rolling_buffer = false;                        // [agimus] rolling_buffer (B7)
};

struct TransformInput { FrameName parent, child; };  // a frame pair, setup only
// One entry per OcpModels::input_transforms, sized at setup. An empty
// optional is a cleared transform (Python None). std::optional<SE3> never
// allocates.
using InputTransforms = std::vector<std::optional<pinocchio::SE3>>;
```

The builder creates `geometry` (a copy of `RobotModels::collision_model()`)
and passes the same pointer to the residuals through `BuildContext` and to
`Ocp` through `OcpModels`, so the pairs a residual adds are the ones `Ocp`
updates.

Never share one action-model object between two nodes. Every node gets its
own object, because references differ per node.

### `Ocp` (`ocp.hpp`)

`Ocp(OcpModels, const OcpParams&)`

- Setup:
  - creates `crocoddyl::ShootingProblem` (x0 zeros of size nx + force_dim);
  - `problem->set_nthreads(n_threads)`;
  - creates the solver in one free function,
    `make_solver(problem, params)`, which returns a
    `std::shared_ptr<crocoddyl::SolverAbstract>`. Today it creates
    `mim_solvers::SolverCSQP` and applies every setting as in
    `PY/ocp_base_croco.py:64-81`. This is the only place that names CSQP.
    `Ocp` reads xs and us through `SolverAbstract`, and the gains through a
    pointer to the solver's `K` vector taken at setup. Another solver (T10.6)
    then means a new branch in `make_solver`, and nothing else changes;
  - sizes results and debug data;
  - with `callbacks == true`, adds the verbose callback. Callbacks print, so
    they are not RT; document this.
- `/// @rt Status set_reference_weighted_trajectory(const std::vector<const WeightedTrajectoryPoint*>& horizon)`
  applies `running_updaters[i]` with `horizon[i]` and `terminal_updater` with
  `horizon.back()`. The size must be `n_controls + 1` (else
  `kSizeMismatch`). Rolling (`OcpModels::rolling_buffer`, B7) is implemented in T8.3; Python
  has it in both the generic and the force-feedback OCP.
- `/// @rt bool solve(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& xs, const std::vector<Eigen::VectorXd>& us)`
  solves with `solver_iters` and `max_solve_time`, then copies the solver's
  xs, us and K into the results and, if `use_debug_data`, fills the debug
  data (`PY/ocp_base_croco.py:134-183`, `PY/ocp/ocp_croco_generic.py:827-853`).
- `bool solve_without_limits(...)` takes the same arguments with 1000
  iterations and no timeout. It is the initial solve, so it is setup, not
  `@rt`.
- `/// @rt void integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Eigen::VectorXd& x_next)`
  calls `calc` on running model 0 and its data.
- `results()`, `debug_data()`, `n_controls()`, `dt()`, `nx()`, `nu()`,
  `problem()`, `solver()`. `solver()` returns the
  `crocoddyl::SolverAbstract`; solver-specific settings are applied in
  `make_solver` from `OcpParams`.
- Geometry:
  - `pinocchio::GeomIndex geometry_index(std::string_view name, pinocchio::GeometryType)`
    is setup and throws on an unknown name.
  - `/// @rt void set_geometry_placement(pinocchio::GeomIndex, const pinocchio::SE3&, pinocchio::GeometryType)`
    updates `OcpModels::geometry`, which is also the colmpc state's
    geometry model when that state is used (`PY/ocp_base_croco.py:110-133`).
- Input transforms (visual servoing):
  - `input_transforms()` returns the list of (parent, child) name pairs (setup);
  - `/// @rt void set_input_transform(std::size_t i, const pinocchio::SE3&)`;
  - `/// @rt void clear_input_transform(std::size_t i)`, which is Python's `None`.

### Cost updaters (`cost_updater.hpp`)

Python dispatches on dataclass type every tick. C++ resolves everything once
at setup: it finds the cost by name, `dynamic_pointer_cast`s the residual and
activation, resolves frame ids, and stores raw pointers plus a rule.

```cpp
enum class UpdateRule {
  kState, kControl, kControlGrav,
  kFramePlacement, kFrameTranslation, kFrameRotation, kFrameVelocity,
  kVisualServoing, kCollisionWeight, kSoftContactForce
};
struct CostUpdater {
  UpdateRule rule;
  crocoddyl::CostItem* item;                       // weight, for kCollisionWeight
  crocoddyl::ResidualModelAbstract* residual;      // static_cast per rule
  crocoddyl::ActivationModelWeightedQuad* activation;  // nullptr if not weighted-quad
  FrameName frame_name;                            // fixed frame, or cache key
  pinocchio::FrameIndex frame_id;
  bool frame_from_trajectory = false;              // Python non-Static residuals
  std::size_t transform_index;                     // kVisualServoing
};
struct NodeUpdater { std::vector<CostUpdater> costs; /* + soft contact in P8 */ };
/// @rt Status apply(const NodeUpdater&, const WeightedTrajectoryPoint&, const InputTransforms&);
```

Rules (Python source: `PY/ocp/ocp_croco_generic.py`, the `update` methods):

| Rule | Reference ← | Activation weights ← | Python lines |
|---|---|---|---|
| kState | `[q; qdot]` | `[w_q; w_qdot]` | 158-160 |
| kControl | `robot_effort` | `w_robot_effort` | 174-176 |
| kControlGrav | (none, see B4) | `w_robot_effort` | 189-191 |
| kFramePlacement | `ee.pose` | `ee.w_pose` | 203-239 |
| kFrameTranslation | `ee.pose.translation()` | `ee.w_pose.head<3>()` | 257-293 |
| kFrameRotation | `ee.pose.rotation()` | `ee.w_pose.tail<3>()` | 311-347 |
| kFrameVelocity | `ee.twist` | `ee.w_twist` | 375-422 |
| kVisualServoing | `T * ee.pose`, where `ee` is `robot_frame + "_vs"` and `T` the input transform (identity if cleared) | `ee.w_pose` | 454-476 |
| kCollisionWeight | (none) | the cost item's scalar `weight = w_collision_avoidance` | 715-719 |

- Which `ee`:
  - With a fixed frame (Python `*Static`), `ee = find_ee_input(p, frame_name)`;
    a missing name returns `kUnknownFrame`.
  - With `frame_from_trajectory` (Python non-Static, `id: 0`), the point must
    hold exactly one `ee_input` (else `kSizeMismatch`), and the residual's
    frame id follows that input's name. Resolve the name to a frame id only
    when it differs from the cached `frame_name`. That lookup scans the
    model's frames, which is bounded and allocation-free (loop over
    `model.frames`, compare `name` with `std::string_view`). It returns
    `kUnknownFrame` if the frame does not exist.
- `kVisualServoing` with a cleared transform and non-zero weights returns
  `kMissingTransform`. This is Python's assert at line 466.
- Applying weights when the activation is not `ActivationModelWeightedQuad`
  does nothing for the weights (Python only defines `update` on WeightedQuad).
- **Zero weight disables the cost (B10).** After writing the weights, the
  updater sets the cost item inactive when its effective weight is zero:
  - the cost item's scalar weight is 0 (`kCollisionWeight`), or
  - every activation weight written by the rule is 0.

  It sets the item active again as soon as one weight is non-zero. A zero
  cost adds nothing to the value, gradient or Hessian, so the solution is
  unchanged, but crocoddyl then skips its `calc`/`calcDiff`. Costs that are
  zero in the OCP file (demo 08's terminal `state_reg`) are made inactive at
  setup in the same way.
  - **RT constraint:** crocoddyl 3.2's `CostModelSum::changeCostStatus`
    allocates: it inserts into and erases from a `std::set<std::string>`,
    and the call builds a `std::string`. The updater must therefore not
    call it in the loop. It writes `CostItem::active` through the cached
    `CostItem*` instead, which is what `calc`/`calcDiff` read.
  - That leaves `CostModelSum::nr_` and the active/inactive name sets
    stale. T4.2 must check, by a test that toggles an item every tick,
    that nothing in the solve path reads them (an RT test plus a golden
    comparison with the same cost disabled through `changeCostStatus` at
    setup). If something does read them, stop (R3) and propose a crocoddyl
    patch: a toggle by index that keeps `nr_` in sync and does not
    allocate.
  - Constraints are never toggled at run time: `changeConstraintStatus`
    changes `ng`, which resizes the solver's workspace. A constraint has no
    weight, so the rule does not apply to it.

### Builders (`ocp_builders.hpp`)

- `OcpModels build_goal_reaching_models(const RobotModels&, const OcpParams&)`
  ports `PY/ocp/ocp_croco_goal_reaching.py`, including its cost names
  (`stateReg`, `ctrlReg`, `goalTracking`) and its updater semantics
  (`set_reference_weighted_trajectory`, lines 125-212), expressed as
  `NodeUpdater`s.
- OCP files (P7, `ocp_definition.hpp`; the `[agimus]` keys are in
  ocp_files.md):

  ```cpp
  struct VisualServoingSpec { std::string cost, world_frame, object_frame; };
  struct NodeDefinition {                 // [agimus.running] / [agimus.terminal]
    std::vector<std::string> update, update_frame_from_trajectory,
        collision_weight_from_trajectory, publish_residuals;  // publish_residuals: running only
    std::vector<VisualServoingSpec> visual_servoing;
  };
  struct OcpDefinition {                  // the parsed [agimus] table
    std::string running_model, terminal_model;
    NodeDefinition running, terminal;
    bool rolling_buffer = false;
  };
  struct OcpFile { crocoddyl_toml::Document document; OcpDefinition definition; };
  // Setup. Registers the CPP/ types (once), loads and validates the file,
  // parses [agimus]. Throws crocoddyl_toml::ParseError or std::invalid_argument.
  OcpFile load_ocp_file(const std::filesystem::path&);
  ```
- `OcpModels build_models_from_toml(const OcpFile&, const RobotModels&, const OcpParams&)`
  (P7) is described in ocp_files.md.
- Both apply `robot_models.armature()` to every differential model, and set
  the per-node dt from `OcpParams::timesteps` and dt 0 on the terminal model.

## Warm starts (`warm_start.hpp`)

```cpp
class WarmStart {
 public:
  virtual ~WarmStart() = default;
  /// @rt
  virtual Status generate(const RobotState& x0_state,
                          const std::vector<const WeightedTrajectoryPoint*>& horizon,
                          Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
                          std::vector<Eigen::VectorXd>& us) = 0;
  /// @rt  (copies into storage sized at setup)
  virtual void update_previous_solution(const OcpResults&);
};
```

- `WarmStartReference(const pinocchio::Model&)` ports
  `PY/warm_start_reference.py`. It uses RNEA on `q, qdot, qddot`; the first
  point is the measured state, so `RobotState::a` is used.
- `WarmStartShiftPreviousSolution(const RobotModels&, const OcpParams&)`
  ports `PY/warm_start_shift_previous_solution.py`. It builds its own Euler
  integrator over a free-forward-dynamics DAM with empty costs and the
  armature. It shifts its own copy of the previous solution in place.
  Without a previous solution it returns `kNoPreviousSolution`.
- `WarmStartShiftPreviousSolutionForceFeedback` (P8) appends `contact_force`
  restricted to the enabled directions to x0.

## MPC (`mpc.hpp`)

`Mpc(std::shared_ptr<Ocp>, std::shared_ptr<WarmStart>, std::shared_ptr<TrajectoryBuffer>)`

- `/// @rt Status run(const RobotState&, std::chrono::nanoseconds now)` is
  `PY/mpc.py:32-66`:
  - return `kNotEnoughData` unless `buffer.has_horizon()` (B2);
  - set the references;
  - generate the warm start into member storage;
  - solve;
  - update the previous solution;
  - `clear_past`;
  - if `use_debug_data` (opt-in, as in Python): fill the debug data and
    the phase timers. With it off, no debug work and no clock read is done
    in the tick.

  Results are read with `ocp().results()`.
- `/// @rt Status integrate(RobotState& state, const Eigen::VectorXd& u)`
  advances q, v and `time` by one dt (B1); `time` is converted from the
  `double` step at this boundary.
- `/// @rt Status push_back(const WeightedTrajectoryPoint&)` forwards to the
  buffer.
- `debug_data()`, `ocp()`, `buffer()`.

### What Python does over a run

- **Start-up, setup:** the ROS node waits until the buffer holds twice the
  horizon, then does one solve warm-started by `WarmStartReference` with no
  iteration limit, and seeds the shift warm start with its result
  (`ROS/agimus_controller.py:368-390,465-481`). C++: `Ocp::solve_without_limits`
  plus `WarmStart::update_previous_solution`, both setup.
- **Buffer running low, in the loop:** below 1.5 horizons the node warns;
  below 1 horizon it logs an error and **appends a copy of the last point**
  (`ROS/agimus_controller.py:497-512`). The reference holds the last point,
  the MPC keeps solving, and the buffer never runs dry. `MPC.run` itself
  returns `None` when data is missing (C++: `kNotEnoughData`).
- The warm start shifts its previous solution for the whole run.

## Performance

Ruling R8. The hot path is everything reachable from `Mpc::run` and
`Mpc::integrate`. Most of a tick is spent in crocoddyl's `calc`/`calcDiff`
and in the CSQP iterations. Our code must not add measurable time on top,
and must not make the solver work harder.

Hot-path rules, on top of the RT rules:

1. **Do each piece of work once per tick.**
   - The warm start writes straight into the `xs`/`us` storage that `Mpc`
     passes to `Ocp::solve`, with no intermediate vectors.
   - `Ocp` copies the solver's xs, us and K into `OcpResults` once. If
     T10.2 shows this copy matters, the results become read-only views of
     the solver's storage; the API keeps `results()` either way.
2. **Pay for debug output only when it is enabled.** With
   `use_debug_data == false`, no reference is converted (no
   `SE3ToXYZQUAT`) and no residual is copied. When enabled, references are
   read from node 0 only, as in Python.
3. **Updaters do no lookups in steady state.** Everything is resolved at
   setup. A trajectory-driven frame costs one `FrameName` comparison per
   tick, and a full scan only when the name changes. An updater writes each
   reference and weight vector once per node.
4. **Use fixed-size Eigen types for fixed dimensions:** `Vector6d` weights,
   `Vector3d`, `SE3`, `Motion`, `Force`. Use `noalias()` on every product
   assigned into existing storage.
5. **Leave solver internals alone.** Do not re-create models, datas or the
   problem after setup. Do not call `calc`/`calcDiff` outside the solver,
   except `Ocp::integrate` (one node).
6. **One object per node, allocated separately.** Every node has its own
   action model, differential model, cost sum, residuals and datas: no
   object is shared between nodes. This is required for correctness (each
   node has its own references) and is expected to help OpenMP, because
   threads working on different nodes then write to different memory. A
   further step is to allocate each node's datas from the thread that will
   compute it (first touch), or with padding to separate cache lines. Do
   this only if T10.4 measures a gain with `n_threads > 1`; it is not done
   up front.
7. **Production builds:** `CMAKE_BUILD_TYPE=Release`,
   `AGIMUS_CONTROLLER_RT_CHECKS=OFF`, generic x86-64. ISA-specific flags
   arrive only through T10.7. Never benchmark an RT-check build:
   `EIGEN_RUNTIME_NO_MALLOC` and the assertions it enables distort timings.

### Benchmark

`CPP/benchmark/bench_mpc.cpp` is a plain executable, not a test: it never
fails on time.

- **Loop.**
  - Build the OCP, do the initial solve without limits, then 50 warm-up
    ticks.
  - Then 1000 measured ticks of `Mpc::run`. The state advances with
    `Mpc::integrate` from the first control, so the loop is closed and
    deterministic.
  - Use the trajectory of golden set G10a.
- **Report** one Markdown table row per OCP and configuration: median,
  p90, p99 and max of the tick; the median of each `MpcDebugData` phase
  timer; mean solver iterations and QP iterations. Also record the git
  commit, build type, compiler, `n_threads`, and the CPU model from
  `/proc/cpuinfo`.
- **Baselines.**
  - A bare-crocoddyl run: the same models from the same builder, with
    `ShootingProblem` plus the same solver called directly, warm-started by
    shifting the previous solution. The difference with `Mpc::run` is the
    agimus overhead (T10.2 target ≤ 2 %).
  - The Python `MPC.run` on the same problem, from a script next to the
    golden generator.
- **Timing.** `std::chrono::steady_clock`, the standard clock for measuring
  intervals (monotonic; `CLOCK_MONOTONIC` on Linux, nanosecond resolution),
  stored as `std::chrono::nanoseconds`; converted to microseconds or
  milliseconds only when the table is printed.
- **Conditions.** Set `n_threads` explicitly and export
  `OMP_THREAD_LIMIT`; some builds default to one OpenMP thread per core. Run
  on a machine that is otherwise idle, at the highest priority:
  `nice -n -20` (needs root or `CAP_SYS_NICE`; also try `chrt -f 99`,
  real-time FIFO scheduling). `taskset` to one
  core is optional. Record all of these with the numbers.
- **OpenMP step** (`n_threads > 1`, T10.4). Before measuring, check and
  record the process affinity (`taskset -p`) and the OpenMP placement:
  `OMP_DISPLAY_ENV=true` and `OMP_DISPLAY_AFFINITY=true` print it. Then
  compare settings for the best tick: `OMP_PROC_BIND` (`close`, `spread`),
  `OMP_PLACES` (`cores`, `threads`), `OMP_WAIT_POLICY=active`, and the
  runtime's spin settings (`GOMP_SPINCOUNT` for GCC's libgomp).
- **Optional, after the port (T10.8):** build the whole stack with GCC and
  with Clang (they ship different OpenMP runtimes, libgomp and libomp) and
  with Intel oneAPI, and compare. It needs a new environment with the whole
  stack rebuilt at full optimisation, so it waits until the code can be
  profiled.
- **Machine.** Numbers that decide anything (R8, T10.x) come from a desktop
  or server CPU, never a laptop. Laptops throttle with temperature, and
  their hybrid performance/efficiency cores make thread timings depend on
  the scheduler. The supervisor asks the user for the machine (a
  workstation such as miyanoura was offered by a reviewer; access goes
  through whoever administers it). A laptop is fine for checking that
  `bench_mpc` runs.

## Behaviour changes from Python

Every deliberate difference. A test covers each one, and the README lists them.

| Id | Python | C++ |
|---|---|---|
| B1 | `PY/mpc.py:77`: `time_ns += int(dt * 1e-9)` always adds 0 | adds `dt`, converted from seconds to `std::chrono::nanoseconds` |
| B2 | `PY/mpc.py:38` checks `len < n_controls + 1`, but `horizon` needs `len > horizon_indexes[-1]` (larger when a factor > 1), so it asserts | checks `has_horizon()` and returns `kNotEnoughData` |
| B3 | `PY/trajectory.py:308-311` raises `KeyError` when a frame is in only one point | a missing side counts as zero weights |
| B4 | `ResidualModelControlGrav.update` sets a `reference` attribute that crocoddyl's class does not have (no effect) | only the weights are updated; see the note below the table |
| B5 | `MPC.append_trajectory_points` calls `buffer.extend`, which does not exist | not ported; call `push_back` in a loop |
| B6 | `ocp_traj_tracking_collision_avoidance.yaml` uses `collision_pair_id`, which the dataclass rejects | the converted file uses geometry names |
| B7 | the rolling buffer (`PY/ocp/ocp_croco_generic.py:865-882`) has three defects, listed below the table | see below |
| B8 | `use_filter_line_search` is a class attribute, not a dataclass field, so the constructor cannot set it | a normal field, default false |
| B9 | `TrajectoryBuffer` grows without bound | fixed capacity; `push_back` returns `kBufferFull` |
| B10 | a cost with zero weight is still evaluated | the updater disables it (see Cost updaters); same solution, less work |

**B4, ControlGrav reference.** crocoddyl's `ResidualModelControlGrav` is
`r = u - g(q)`. It has no reference, so Python's
`obj.reference = robot_effort` creates a Python attribute that crocoddyl
never reads. The effective Python behaviour is "regularise the torque
around gravity compensation, and ignore `robot_effort`", and C++ keeps it,
which keeps parity. If `robot_effort` is meant to be tracked around gravity
compensation (`r = u - g(q) - u_ref`), that needs a new residual, which
changes the solution compared with Python. It is not in this plan, and the
user decides whether to add it. Status (2026-11-25): the user is not sure,
so the plan keeps parity and adds no residual. T7.1 rejects nothing, and the
README says that `update` on `control_grav` updates only the weights.

**B7, rolling buffer.** With `expect_rolling_buffer`, Python sets the
references of every node on the first call. On each later call it rotates
node 0 to the end (`circularAppend`) and sets the references of the new
last node only. The defects:

1. **dt moves with the node.** Node 0's dt goes to the end, which is wrong
   when the timesteps are not all equal.
2. **One node per tick is assumed, not checked.** Rotating is correct only
   if, since the last tick, the buffer has advanced by exactly one node and
   the points already in the horizon have not changed. It goes wrong when a
   tick is skipped, when the MPC rate differs from `1 / dt`, when a
   dt factor is greater than 1, or when the trajectory is replaced.
3. **The one-argument `circularAppend(model)` calls `model->createData()`
   every tick**, which allocates (crocoddyl 3.2
   `optctrl/shooting.hxx`). The solver also calls `resizeData()` on the next
   solve, because `circularAppend` sets `is_updated_`.

C++:

- Python's `expect_rolling_buffer` is the `[agimus]` key `rolling_buffer`,
  carried by `OcpModels::rolling_buffer`. Setup throws if it is set and a
  dt factor is not 1
  (fixes 1). Equal factors greater than 1 are not enough: node 1 then sits
  at buffer index 2, but the buffer advances by one point per tick.
- `Ocp` remembers the buffer position of node 0 from the previous tick (the
  `id` of the point used). It rotates only when the new node 0 is the old
  node 1. Otherwise (first call, skipped tick, jump) it updates every node,
  exactly as without the rolling buffer (fixes 2).
- It rotates with the two-argument `circularAppend(model, data)`, passing
  node 0's own data, so nothing is created (fixes 3). `RtOcp.set_reference...`
  covers a rotation followed by a solve, so the `resizeData()` that the
  solver then runs is proven allocation-free (same sizes), or goes red (R3).
- Benchmark (T10.2): rolling against updating every node, on the same
  problem; keep rolling only if it is measurably faster (R8).
- Test: over 50 ticks with a skipped tick and a trajectory jump, the
  references of every node equal those of an OCP without the rolling
  buffer.
- **Ruling (user, 2026-11-25): keep the rolling buffer.** Rolling
  (`set_reference_weighted_trajectory`) is `@rt`, with its `Rt*` test. No
  reset is planned.

## Golden files

Generated by `CPP/test/golden/generate_golden.py` in the Python judge
environment. Each set is one TOML file (written with `tomli-w`, read with
toml++; TOML floats round-trip exactly) holding inputs and outputs as lists of
numbers. The robot is example-robot-data's `panda`, with
`moving_joint_names` = the 7 arm joints and armature 0.1, unless stated.

| Set | Python under test | Content |
|---|---|---|
| G1 | `OCPParamsBaseCroco` | timesteps, n_controls, total_time for `[1]/[19]`, `[1,2]/[20,10]`, `[1,2,4,8]/[5,5,5,4]` |
| G2 | `WarmStartReference.generate` | x0, xs, us for a 5-point reference (ur5, as its Python test) |
| G3 | `WarmStartShiftPreviousSolution.generate` | shifted xs, us with factors `[1,2]` |
| G4 | `test_ocp_croco_base.py` | its `simple_ocp_croco_results.pkl` converted, plus the problem; its running models use crocoddyl's default 1 ms step, not `dt` |
| G5 | `OCPCrocoGoalReaching` | x0, reference, solution xs, us, K[0], kkt, iterations |
| G6 | `OCPCrocoGeneric(ocp_goal_reaching.yaml)` | same content as G5 |
| G7 | `OCPCrocoGeneric` with demo 08's collision OCP | solution plus collision residuals |
| G8 | `OCPCrocoGeneric` with visual servoing | solution with a given transform, and the `kMissingTransform` case |
| G9 | `OCPCrocoForceFeedbackGeneric` (demo 07-like) | solution with 1D and 3D soft contact |
| G10a/b | `MPC.run` closed loop, 20 ticks | per tick: u0, K0, nb_iter (a: goal-reaching class, b: TOML file) |

Tolerances: 1e-10 for kinematics and RNEA; 1e-6 absolute for solver outputs.
Only the supervisor may loosen a tolerance, writing the reason in the test
and the ledger. If the generator needs the B1/B2 fix to produce the closed
loop, it applies the fix in the script, not in the Python package.
