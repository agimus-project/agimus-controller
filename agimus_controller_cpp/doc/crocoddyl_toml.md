# crocoddyl-toml: OCP descriptions in TOML

Ruling R2 (2026-11-25): OCPs are described in TOML and loaded by
**crocoddyl-toml**, a new library in its own repository (`CT/`).

- The file format is designed for TOML. It is not a copy of crocoddyl_yaml's
  schema.
- Files are validated by typed C++ structs, not by schema files.
- crocoddyl_yaml is not upgraded, extended or used.

## Acknowledgment: crocoddyl_yaml

crocoddyl-toml starts from **crocoddyl_yaml** (PAL GitLab
`control/crocoddyl_yaml`, written by PepMS between 2024-12-16 and
2024-12-20). That library showed that a crocoddyl OCP can be described in a
data file and built by one factory per model family:

- actuation;
- activation;
- residual;
- cost item;
- constraint;
- contact;
- differential and integrated action model;
- problem.

crocoddyl-toml keeps that architecture and its idea of reusable stages. It
re-implements the rest:

- TOML instead of YAML;
- typed structs instead of yaml-schema-cpp;
- the crocoddyl 3 and pinocchio 4 APIs;
- a registry for types defined outside the library.

crocoddyl_yaml's type list is the starting point for the built-in types
below.

Reviewing crocoddyl_yaml for this plan turned up defects. Each becomes a
requirement here, with a test:

| Found in crocoddyl_yaml | Requirement in crocoddyl-toml |
|---|---|
| The contact case falls through to `default: throw` (missing `break`, `src/factories/diff_action.cpp:92-96`) | Every built-in type has a test that builds it |
| Every node of a stage, and the terminal node, share one action-model object (`src/factories/problem.cpp:58-83`) | Every `build_*` call returns new objects; nothing is shared between nodes |
| `integrator` is parsed and then discarded (`src/trajectory.cpp:83-89`) | Every key is either used or rejected; a test per key |
| The node count `int(duration / dt)` truncates (`0.19 / 0.01` gives 18) | Times are integer `std::chrono::nanoseconds`; node counts are given, not computed |
| Error messages copied between factories ("Residual ContactFrictionCone" in contacts) | Error messages are generated from the key path, never written by hand |
| Dead, uncompilable files; `ProblemFactory` without a test | `-Werror` in CI; every public function has a test |

The YAML-versus-TOML decision came from the gepetto maintainers (PR #334).
YAML 1.1's implicit typing turns `NO` into false and `3.10` into 3.1, YAML
parsers disagree with each other, and Python's standard library reads TOML
(`tomllib`) but not YAML.

## Repository and build

- Repository: [github.com/loco-3d/crocoddyl-toml](https://github.com/loco-3d/crocoddyl-toml)
  (shorthand `CT/`), next to crocoddyl. It was created empty on 2026-11-25,
  with no license yet; crocoddyl is BSD-3-Clause, and the user picks the
  license.
- **Packaging follows the Gepetto/loco-3d packages, not R6.** R6's plain
  CMake applies to `CPP/` only. The template is
  [gepetto/example-adder](https://github.com/gepetto/example-adder):
  - `CMakeLists.txt` with jrl-cmakemodules found as an installed package
    (`find_package(jrl-cmakemodules REQUIRED CONFIG)`, then `base.cmake`),
    not a git submodule. It uses `PROJECT_NAME crocoddyl-toml`,
    `PROJECT_URL https://github.com/loco-3d/crocoddyl-toml`,
    `PROJECT_USE_CMAKE_EXPORT TRUE`,
    `compute_project_args`, `check_minimal_cxx_standard(17 ENFORCE)`, and
    jrl's install and export. The container has jrl-cmakemodules 2.3.0
    (`ros-lyrical-jrl-cmakemodules`).
  - `package.xml` format 3 with `build_type` cmake and
    `build_depend jrl_cmakemodules`, so colcon and the ROS buildfarm can
    build it.
  - `flake.nix` with `inputs.gepetto.url = "github:gepetto/nix"` and
    `gepetto.lib.mkFlakoboros`, overriding the package's `src` with the
    local file set; `.envrc` with `use flake .`. The override needs the
    package to exist in gepetto/nix, so T5.1 also writes
    `pkgs/crocoddyl-toml.nix` for gepetto/nix (modelled on
    `pkgs/force-feedback-mpc.nix`). Opening that pull request is outward
    facing: the user approves it first.
  - The CI workflows, `.pre-commit-config.yaml` and `.mergify.yml` are
    copied from example-adder (`nix.yml`, `ci_ros2.yml`,
    `update-flake-lock.yml`).
  - Tests use doctest (R6's test rule), registered with jrl's
    `add_unit_test`. Gepetto packages usually use Boost.Test; if the
    loco-3d maintainers ask for it, switch before T5.2, while there are few
    tests.
  - No Python bindings in this plan (R1). The layout reserves `python/`
    and the `BUILD_PYTHON_INTERFACE` option (default OFF until bindings
    exist), because crocoddyl's users are mostly in Python and the
    maintainers will likely ask for them.
- Dependencies: crocoddyl, pinocchio, Eigen and toml++ 3.4. toml++ is
  packaged as `libtomlplusplus-dev` in Ubuntu and as `tomlplusplus` in
  nixpkgs.
  - It has no dependency on colmpc, force_feedback_mpc or agimus. Those
    register their types from outside (see Registry).
  - toml++ is header-only and heavy to compile. Compile it once, in
    `src/toml_impl.cpp` with `TOML_HEADER_ONLY=0`, so every translation unit
    stays under 2 GB.
- **Every name comes from the package name**, never from the GitHub
  organisation (which appears only in `PROJECT_URL`):
  - CMake project and exported target: `crocoddyl-toml::crocoddyl-toml`
    (jrl's `PROJECT_NAME::PROJECT_NAME`);
  - headers: `include/crocoddyl-toml/` (jrl's `CUSTOM_HEADER_DIR`);
  - C++ namespace: `crocoddyl_toml`, the package name with `-` replaced by
    `_`, because a C++ identifier cannot contain `-`.

## File format

Principles:

- **Named things are tables keyed by their name**, not arrays of tables:
  `[models.running.costs.state_reg]`, not `[[stages.costs]]`. crocoddyl
  stores costs and constraints by name anyway, so order does not matter.
- **Short sub-objects are inline tables** that fit on one line (TOML 1.0
  inline tables cannot span lines): `residual = { type = "state" }`. Longer
  ones use a sub-table, `[models.running.costs.x.residual]`.
- **Every object with variants has a `type` key**, written in `snake_case`.
- **Times are seconds** (`dt = 0.01`). The loader converts them to
  `std::chrono::nanoseconds` with `std::chrono::round`.
- **Infinite bounds use TOML's native `inf` and `-inf`.**
- **A vector can be given as a scalar**, which is broadcast to the size the
  model needs, as in Python (`weights = 1.0`).
- **Poses** are `{ translation = [x, y, z], quaternion = [x, y, z, w] }`. The
  quaternion order is written out in the docs because it differs between
  libraries. Identity by default.
- **Strict parsing.** An unknown key, a wrong type or a wrong size is an
  error. The error message gives the file, line, column and key path:
  `goal.toml:14:1: models.running.costs.goal_tracking.activation.weights:
  expected 6 values (nr of frame_placement), got 3`.
- **Versioned.** The first key is `format = 1`, and the loader rejects other
  versions. The translator from the Python agimus format (later, separate)
  writes this format.

Example: the goal-reaching OCP with its agimus section.

```toml
format = 1

[models.running]
dynamics = { type = "free_fwd" }
integrator = { type = "euler" }
actuation = { type = "full" }

[models.running.costs.control_reg]
weight = 1.0
residual = { type = "control" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.running.costs.state_reg]
weight = 1.0
residual = { type = "state" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.running.costs.goal_tracking]
weight = 1.0
# Python `id: 0`: the frame follows the trajectory ([agimus] below).
residual = { type = "frame_placement", frame = "universe" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.terminal]
dynamics = { type = "free_fwd" }
integrator = { type = "euler" }
actuation = { type = "full" }

[models.terminal.costs.state_reg]
weight = 1.0
residual = { type = "state" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.terminal.costs.goal_tracking]
weight = 1.0
residual = { type = "frame_placement", frame = "universe" }
activation = { type = "weighted_quad", weights = 1.0 }

# Read by agimus_controller_cpp, ignored by crocoddyl-toml (see below).
[agimus]
running_model = "running"
terminal_model = "terminal"

[agimus.running]
update = ["control_reg", "state_reg"]
update_frame_from_trajectory = ["goal_tracking"]

[agimus.terminal]
update = ["state_reg"]
update_frame_from_trajectory = ["goal_tracking"]
```

An optional `[problem]` table describes a whole horizon for users that do
not build it themselves (agimus builds its own, from `OcpParams`):

```toml
[problem]
nodes = [
  { model = "running", count = 20, dt = 0.01 },
  { model = "running", count = 10, dt = 0.02 },
]
terminal = "terminal"
```

### Application tables

crocoddyl-toml parses only `format`, `models` and `problem`. It returns every
other top-level table untouched in `Document::extra`, so an application keeps
its own keys in the same file. agimus uses `[agimus]`. This replaces the
earlier "small agimus YAML file": one OCP is one file.

## C++ API

```cpp
namespace crocoddyl_toml {

struct Document {
  std::map<std::string, ModelSpec, std::less<>> models;
  std::optional<ProblemSpec> problem;
  toml::table extra;                       // application tables, untouched
  std::filesystem::path source;            // for error messages
};

// Setup only. Throws ParseError (a std::runtime_error) with file:line:col and
// the key path. Validates every key, including registered types' keys.
Document load_file(const std::filesystem::path&);
Document load_string(std::string_view toml, std::string_view source_name);

struct BuildContext {
  std::shared_ptr<crocoddyl::StateMultibody> state;
  std::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;  // nullptr: from the model's `actuation`
  std::shared_ptr<pinocchio::GeometryModel> geometry;            // nullptr if unused
};

// New objects on every call (one per node). Throws BuildError (key path) on
// an unknown frame or a size mismatch.
std::shared_ptr<crocoddyl::ActionModelAbstract> build_model(
    const Document&, std::string_view model, const BuildContext&,
    std::chrono::nanoseconds dt);
std::shared_ptr<crocoddyl::ShootingProblem> build_problem(
    const Document&, const BuildContext&, const Eigen::VectorXd& x0);

}  // namespace crocoddyl_toml
```

- `ModelSpec`, `CostSpec`, `ConstraintSpec` and one struct per built-in type
  (`StateResidualSpec { std::optional<Eigen::VectorXd> xref; }`, ...) hold
  the parsed values. Each has a `parse(const toml::table&, KeyPath)`
  function. Defaults live in the struct definition.
- Sizes that depend on the robot (nv, nr) are checked in `build_model`,
  where they are known. Everything else is checked in `load_*`.

### Registry

For types defined outside crocoddyl-toml (colmpc, force_feedback_mpc):

```cpp
struct ResidualType {
  std::function<void(const toml::table&, const KeyPath&)> validate;   // at load
  std::function<ResidualModelAbstractPtr(const toml::table&, const BuildContext&)> build;
};
void register_residual(std::string type, ResidualType);
void register_activation(std::string type, ActivationType);
void register_dynamics(std::string type, DynamicsType);
void register_integrator(std::string type, IntegratorType);
```

- Registration is setup-only and not thread-safe: register before loading
  files. Registering a name twice, or a built-in name, throws.
- An unknown `type` is a `ParseError` that lists the known types for that
  family.

## Built-in types

These are the types agimus needs, all plain crocoddyl. The rest of
crocoddyl_yaml's list (rigid contacts 1D–6D, friction cone, frame axis
alignment, frame collision, smooth norms, flat exp/log activations, RK
integrators) is added when someone needs it, each with its test.

| Family | `type` | crocoddyl class | Keys (default) |
|---|---|---|---|
| dynamics | `free_fwd` | `DifferentialActionModelFreeFwdDynamics` | none |
| integrator | `euler` | `IntegratedActionModelEuler` | `with_cost_residual` (true) |
| actuation | `full` | `ActuationModelFull` | none |
| actuation | `floating_base` | `ActuationModelFloatingBase` | none |
| residual | `state` | `ResidualModelState` | `xref` (neutral, zero velocity) |
| residual | `control` | `ResidualModelControl` | `uref` (zero) |
| residual | `control_grav` | `ResidualModelControlGrav` | none |
| residual | `frame_placement` | `ResidualModelFramePlacement` | `frame`, `pose` (identity) |
| residual | `frame_translation` | `ResidualModelFrameTranslation` | `frame`, `translation` (zero) |
| residual | `frame_rotation` | `ResidualModelFrameRotation` | `frame`, `quaternion` (identity) |
| residual | `frame_velocity` | `ResidualModelFrameVelocity` | `frame`, `twist` (zero), `reference_frame` (`"world"`; or `"local"`, `"local_world_aligned"`) |
| activation | `quad` | `ActivationModelQuad` | none (the default when `activation` is absent) |
| activation | `weighted_quad` | `ActivationModelWeightedQuad` | `weights` (scalar or vector of size nr) |
| constraint | `residual` (default) | `ConstraintModelResidual` | `residual`, `lower` (-inf), `upper` (inf), `terminal` (true) |
| constraint | `control_limit` | `ConstraintModelResidual` on a control residual | bounds from the model's `effortLimit` (Python `ConstraintModelControlLimit`) |

Cost item keys: `weight` (1.0), `active` (true), `residual`, `activation`.
Constraint item keys: `active` (true), plus the constraint's keys.

## Types registered by agimus_controller_cpp

They live in `CPP/src/toml_types/` and are registered by
`agimus_controller::register_toml_types()`. Registration must happen before
a file is loaded, because loading validates the registered types' keys. So
agimus loads OCP files only through `agimus_controller::load_ocp_file(path)`
(setup), which:

1. calls `register_toml_types()`, which is idempotent (`std::call_once`), so
   loading several files never registers a name twice;
2. calls `crocoddyl_toml::load_file`;
3. parses `[agimus]` into an `OcpDefinition`;
4. returns both as an `OcpFile` (design.md, Builders), the input of
   `build_models_from_toml`.

| `type` | Family | Builds | Keys | CMake option |
|---|---|---|---|---|
| `colmpc_exp`, `colmpc_quad_exp` | activation | `colmpc::ActivationModelExp` / `QuadExp(nr, alpha)` | `alpha` (1.0) | `AGIMUS_CONTROLLER_WITH_COLMPC` |
| `colmpc_distance_collision` | residual | `colmpc::ResidualDistanceCollision(state, nu, geometry, pair_id)` | `geometry_1`, `geometry_2` (names) | same |
| `colmpc_distance_collision2` | residual | `colmpc::ResidualDistanceCollision2(state, nu, pair_id)` | `geometry_1`, `geometry_2` | same |
| `colmpc_free_fwd` | dynamics | colmpc's free-forward-dynamics DAM | none | same |
| `soft_contact_1d`, `soft_contact_3d` | dynamics | force_feedback_mpc `DAMSoftContact{1D,3D}AugmentedFwdDynamics` | `frame`, `kp`, `kv`, `oPc`, `ref`, `cost_ref`, `with_gravity_torque_reg` (false), `axis` (1D) or `enabled_directions` (3D) | `AGIMUS_CONTROLLER_WITH_FORCE_FEEDBACK` |
| `soft_contact_euler` | integrator | `IAMSoftContactAugmented` | `force_lb`, `force_ub` | same |

Collision rules (port of `PY/ocp/ocp_croco_generic.py:498-550`):

- the residual adds the geometry pair to the geometry model if it is
  missing, then uses its index;
- There is one geometry model per OCP: `BuildContext::geometry`, which
  becomes `OcpModels::geometry`. When colmpc's state is used, agimus creates
  it from that same pointer, so the pairs a residual adds and the placements
  `Ocp::set_geometry_placement` updates are in one object. If colmpc's state
  copies the geometry model instead of sharing it, stop (condition 4).
- `colmpc_distance_collision2` needs colmpc's `StateMultibody` and the
  `colmpc_free_fwd` dynamics. The file says so explicitly with
  `dynamics = { type = "colmpc_free_fwd" }`; nothing is chosen silently, as
  Python did (`PY/.../ocp_croco_generic.py:675-711`). agimus creates
  colmpc's state when a model it builds uses `colmpc_free_fwd`, and building
  `colmpc_distance_collision2` under other dynamics is a `BuildError`.

## The `[agimus]` table

Parsed by `CPP/` (`ocp_definition.hpp/.cpp`) into an `OcpDefinition`, with
the same strict rules: typed structs, unknown keys rejected, key paths in
errors.

| Key | Meaning (Python equivalent) |
|---|---|
| `running_model`, `terminal_model` | names in `[models]` used as the templates of every running node and of the terminal node |
| `<node>.update` | costs whose reference and weights follow the trajectory (`update: true`); the rule comes from the residual type |
| `<node>.update_frame_from_trajectory` | frame residuals whose frame follows the trajectory point's single end effector (non-Static residuals, `id: 0`) |
| `<node>.collision_weight_from_trajectory` | costs whose weight is `w_collision_avoidance` |
| `<node>.publish_residuals` | `publish_residual: true`; `running` only, because Python only publishes running residuals |
| `<node>.visual_servoing` | array of `{ cost, world_frame, object_frame }` (`ResidualModelVisualServoing`) |
| `rolling_buffer` | Python `expect_rolling_buffer` (default false); see design.md, B7 |

`<node>` is `running` or `terminal`. Python's `active: false` is the
crocoddyl-toml `active = false` key on the cost or constraint itself.

Rules:

- A name that is listed but missing from its model is a setup error.
- `update_frame_from_trajectory` and `visual_servoing` imply `update`: the
  cost's reference and weights follow the trajectory too. Listing it in
  `update` as well is allowed and changes nothing.
- A cost listed under `visual_servoing` uses the `kVisualServoing` rule,
  not the rule of its residual type (`frame_placement`). Listing it under
  `update_frame_from_trajectory` too is a setup error.
- Force feedback needs no `[agimus]` key: the soft-contact dynamics and
  integrator types carry its settings, and the soft-contact updater follows
  from them (P8).
- A cost listed under `update` whose residual type has no rule (see the
  updater table in design.md) is a setup error.
- The update rule follows from the residual type, found once at setup
  (`state`, `control`, `control_grav`, `frame_placement`,
  `frame_translation`, `frame_rotation`, `frame_velocity`).
- For a fixed-frame rule, the end-effector input is looked up by the
  residual's own `frame`.
- For visual servoing, the robot frame is the cost's residual `frame` and
  the input key is that name plus `"_vs"` (Python
  `PY/ocp/ocp_croco_generic.py:488-489`). The world frame must have parent
  joint 0 and an identity placement (lines 479-486).
- `[problem]` is ignored: the horizon comes from `OcpParams`.

## Building the horizon from the templates

T7.2, `build_models_from_toml`:

1. Take the `OcpFile` returned by `load_ocp_file` (registration and
   loading are already done). Until T8.3 implements rolling, throw
   `std::invalid_argument` if `rolling_buffer` is true.
2. Create the state: colmpc's if either template uses `colmpc_free_fwd`,
   otherwise crocoddyl's. Use the reduced model from `RobotModels`.
3. For each of the `n_controls` running nodes, call
   `build_model(doc, running_model, ctx, timesteps[i])`, set the armature,
   and build the node's `NodeUpdater` from `[agimus.running]`.
4. The terminal node is the same, from `terminal_model`, with dt 0.
5. Fill `OcpModels::reference_costs`, `residual_costs` and
   `input_transforms`.

## Reference OCP to convert (demo 08, agimus-demos `main`, read at `5586a0e`)

`agimus_demo_08_collision_avoidance/config/ocp_definition_file.yaml`, with
`${arm_id}` → `fer`:

- **Running model:**
  - costs `control_reg` (ControlGrav), `state_reg` (State),
    `target_pose_tracking` (FrameTranslationStatic on `fer_hand`) and
    `target_rotation_tracking` (FrameRotationStatic on `fer_hand`);
  - all with WeightedQuad weights 1.0, cost weight 1.0 and `update: true`;
  - constraint `collision`: ResidualDistanceCollision2 between
    `fer_hand_sc_capsule_0` and `obstacle_capsule_0`, lower 0.01, upper inf.
- **Terminal model:** the same, without `control_reg`; `state_reg` has cost
  weight 0.0.

The test robot is example-robot-data's panda, which does not have these
geometry names. The test version uses a capsule from panda's collision model
(after `collision_as_capsule`) and an obstacle capsule from a small
environment URDF in `CPP/test/resources/`. The golden generator uses the
same files.

The Python originals of the other test OCPs are in this repo
(`PY/ocp/ocp_goal_reaching.yaml`, and
`agimus_controller_examples/.../panda_pick_and_place/config/ocp_definition_file.yaml`).
