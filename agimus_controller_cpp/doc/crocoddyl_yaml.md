# crocoddyl_yaml: upgrade, extensions and the agimus OCP files

Ruling R2: OCPs are described in crocoddyl_yaml files, and **the
crocoddyl_yaml schema is frozen**.

- **Allowed:** new type files (`schemas/residuals/<new>.schema`,
  `schemas/activations/<new>.schema`, `schemas/contacts/<new>.schema`), their
  factory code, and implementing keys the schema already declares.
- **Not allowed:** editing an existing `.schema` file's keys, types or
  defaults. If a task cannot be done without such an edit, stop and ask the
  supervisor, who asks the user.

A translator from the Python YAML format will come later and separately. Our
tests use hand-converted files (T7.3).

## Facts about the repo (`CY/`, PAL GitLab `control/crocoddyl_yaml`)

- 27 commits, all between **2024-12-16 and 2024-12-20**. It targets
  crocoddyl ≥ 2.0 (written against the 2.x API, which used
  `boost::shared_ptr`), pinocchio ≥ 3.0, and yaml-schema-cpp as of December
  2024.
- To migrate, compare against the versions recorded in `doc/environment.md`:
  - **crocoddyl** `CHANGELOG.md` (github.com/loco-3d/crocoddyl), every entry
    after the last 2.x release before 2024-12-20. Expect the switch from
    `boost::shared_ptr` to `std::shared_ptr` (3.0) and renamed or removed
    members.
  - **pinocchio** `CHANGELOG.md` (github.com/stack-of-tasks/pinocchio), from
    3.3 to the installed version.
  - **yaml-schema-cpp** (`src/yaml-schema-cpp`, `git log --since=2024-12-01`),
    for changes to `FoldersSchema`, `derived` and `follow`.

  Use the changelogs to explain each compile error. Do not invent API.
- Build: CMake on jrl-cmakemodules, which is fetched from GitHub at configure
  time when the `cmake` submodule is empty. Python bindings default ON (they
  need eigenpy); pass `-DBUILD_PYTHON_INTERFACE=OFF`. There is no
  `package.xml`.
- Schemas are not installed: `CROCODDYL_YAML_SCHEMA_DIR` points at the source
  tree (`config/path.hpp.in`).

## Defects

Fix each in T5.3 with one commit and one test.

| Defect | Location | Fix |
|---|---|---|
| Contact case falls through to the `default: throw` (missing `break`) | `src/factories/diff_action.cpp:92-96` | add the `break` |
| Every node of a stage shares one action-model object, and the terminal model is that same object | `src/factories/problem.cpp:58-83` | implement the declared `independent_models` key (one object per node when true) |
| `integrator` from YAML is parsed and discarded | `src/trajectory.cpp:83-89` | store it and expose `get_integrator()` |
| `dt` read unconditionally although optional | `src/trajectory.cpp:92` | read only if present; throw a clear error where it is needed |
| Node count `int(duration / dt)` truncates (`0.19/0.01` gives 18) | `src/trajectory.cpp:143-159` | round with a tolerance |
| `test_factory_constraint.cpp` includes missing headers and is not registered | `test/` | rewrite it in T6.4, or delete it in T5.3 if T6.4 is blocked |
| `cost_item.cpp` / `cost_item.hpp` are uncompilable dead code | `src/factories/cost_item.cpp` | delete |
| Copy-pasted error messages (contacts say "Residual ContactFrictionCone") | `src/factories/contact.cpp:143` and others | fix the text |
| `ProblemFactory` has no test | `test/` | add `test_problem.cpp` |

## API additions

T5.4. Keep the existing overloads.

agimus needs to build the models with its own state, because:

- the state is colmpc's `StateMultibody` when a residual needs it;
- the pinocchio model is the reduced model from `RobotModels`;
- the collision residuals need the geometry model.

Add a context struct and overloads that take it:

```cpp
struct BuildContext {
  std::shared_ptr<crocoddyl::StateMultibody> state;
  std::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  std::shared_ptr<pinocchio::GeometryModel> geometry;   // nullptr if no collision types
};
StagePtr Stage::create(const YAML::Node&, const BuildContext&);
ResidualModelAbstractPtr ResidualModelFactory::create(const YAML::Node&, const BuildContext&);
DifferentialActionModelAbstractPtr DifferentialActionModelFactory::create(const StagePtr&, const BuildContext&);
```

The old overloads become thin wrappers that build a `BuildContext` with a
null geometry.

## New types

T6.x and T8.1. Each one is a new schema file, a factory enum entry, a test
YAML under `test/yaml/` and a test case in the repo's existing gtest suite
(doctest applies to `CPP/` only).

| New type (YAML `type:`) | Schema file | Builds | Keys |
|---|---|---|---|
| `control_grav` | `residuals/control_grav.schema` | `crocoddyl::ResidualModelControlGrav` | none |
| `colmpc_exp` | `activations/colmpc_exp.schema` | `colmpc::ActivationModelExp(nr, alpha)` | `alpha` |
| `colmpc_quad_exp` | `activations/colmpc_quad_exp.schema` | `colmpc::ActivationModelQuadExp(nr, alpha)` | `alpha` |
| `colmpc_distance_collision` | `residuals/colmpc_distance_collision.schema` | `colmpc::ResidualDistanceCollision(state, nu, geometry, pair_id)` | `geometry_1`, `geometry_2` (names) |
| `colmpc_distance_collision2` | `residuals/colmpc_distance_collision2.schema` | `colmpc::ResidualDistanceCollision2(state, nu, pair_id)` | `geometry_1`, `geometry_2` |
| `soft_1d`, `soft_3d` (contacts, P8) | `contacts/soft_1d.schema`, `contacts/soft_3d.schema` | force_feedback_mpc `DAMSoftContact{1D,3D}AugmentedFwdDynamics` | `link_name`, `kp`, `kv`, `oPc`, `ref`, `cost_ref`, `with_gravity_torque_reg`, plus `axis` (1D only) |

Collision rules (port of `PY/ocp/ocp_croco_generic.py:498-550`):

- the residual adds the geometry pair to the geometry model if it is
  missing, then uses its index;
- `colmpc_distance_collision2` needs colmpc's `StateMultibody` and colmpc's
  free-forward-dynamics DAM. Expose
  `bool stage_needs_colmpc(const YAML::Node& stage)` so agimus can create the
  right state before building, and make the DAM factory pick colmpc's DAM
  when the stage contains such a residual (`PY/.../ocp_croco_generic.py:675-711`).

colmpc and force_feedback_mpc are optional dependencies, behind
`CROCODDYL_YAML_WITH_COLMPC` and `CROCODDYL_YAML_WITH_FORCE_FEEDBACK`. Using a
type whose option is OFF throws a clear error.

### Declared keys that are expected to need a schema edit

Two keys are declared but cannot validate as written. Expect a stop at these
points; the user decides.

- `stage.schema` → `constraints: constraint[]`. In `constraint.schema`,
  `residual` has `_type: residual`, but no `residual.schema` exists; the
  residual schemas use `_base: residual_base`. `lower_bound` and
  `upper_bound` are mandatory `VectorXd`, so an infinite bound is written
  `.inf`.
- `stage.schema` → `contacts: contact[]`. No `contact.schema` exists.

## The agimus OCP file

T7.1. Each OCP has two files:

- the crocoddyl_yaml trajectory file (schema frozen);
- an agimus file pointing at it, validated by `CPP/schemas/agimus_ocp.schema`
  with yaml-schema-cpp.

```yaml
# goal_reaching.agimus.yaml
crocoddyl_yaml_file: goal_reaching.cy.yaml  # relative to this file, or absolute
running_stage: running                      # stage used as the template of every running node
terminal_stage: terminal                    # stage used for the terminal node
running:
  update: [control_reg, state_reg]          # Python `update: true`; rule from the residual type
  update_frame_from_trajectory: [goal_tracking]  # Python non-Static frame residuals (`id: 0`)
  collision_weight_from_trajectory: [distance]   # cost weight = w_collision_avoidance
  inactive: []                              # Python `active: false` (costs and constraints)
  publish_residuals: [state_reg]            # Python `publish_residual: true`
  visual_servoing:                          # Python ResidualModelVisualServoing
  - cost: vs_tracking                       # a frame_placement cost on robot_frame
    world_frame: world
    object_frame: object
terminal:
  update: [state_reg]
  update_frame_from_trajectory: [goal_tracking]
soft_contact:                               # force feedback only (P8)
  force_lb: [-100.0]
  force_ub: [100.0]
```

Rules:

- A name that is listed but missing from its stage is a setup error.
- A cost listed under `update` whose residual type has no rule (see the
  updater table in design.md) is a setup error.
- The update rule follows from the residual type, found once at setup
  (`state`, `control`, `control_grav`, `frame_placement`,
  `frame_translation`, `frame_rotation`, `frame_velocity`).
- For a fixed-frame rule, the end-effector input is looked up by the
  residual's own `link_name`.
- For visual servoing, the robot frame is the residual's `link_name` and the
  input key is `link_name + "_vs"` (Python
  `PY/ocp/ocp_croco_generic.py:488-489`). The world frame must have parent
  joint 0 and an identity placement (lines 479-486).
- `publish_residuals` exists only in `running`, because Python only
  publishes running residuals.

Keys of the crocoddyl_yaml file that agimus ignores, because the agimus
params decide them:

- `dt`, each stage's `duration` and `state_initial`: timing comes from
  `OcpParams`; x0 comes from the warm start;
- `independent_models`: agimus always builds one object per node.

## Building the horizon from stage templates

T7.2, `build_models_from_yaml`:

1. Load and validate both files. Find the running and terminal stages by
   name.
2. Create the state: colmpc's if `stage_needs_colmpc` is true for either
   stage, otherwise crocoddyl's. Use the reduced model from `RobotModels`.
   Create the actuation from the trajectory's `actuation`.
3. For each of the `n_controls` running nodes:
   - `Stage::create(running_node, ctx)` (a fresh cost sum per call);
   - the DAM from the factory;
   - `armature = robot_models.armature()`;
   - Euler integration with `dt = timesteps[i]`, or `IAMSoftContactAugmented`
     when the stage has a soft contact (P8);
   - apply `inactive` with `CostModelSum::changeCostStatus` (and the
     constraint manager's equivalent);
   - build the node's `NodeUpdater` from the agimus file.
4. The terminal node is the same, from the terminal stage, with dt 0.
5. Fill `OcpModels::reference_costs`, `residual_costs` and `input_transforms`.

The Euler step is the trajectory's `integrator`; only `euler` is required
(Python has only Euler). Other values are a setup error until a test needs
them.

## Reference OCP to convert (demo 08, agimus-demos `5586a0e`)

`agimus_demo_08_collision_avoidance/config/ocp_definition_file.yaml`, with
`${arm_id}` → `fer`:

- **Running stage:**
  - costs `control_reg` (ControlGrav), `state_reg` (State),
    `target_pose_tracking` (FrameTranslationStatic on `fer_hand`) and
    `target_rotation_tracking` (FrameRotationStatic on `fer_hand`);
  - all with WeightedQuad weights 1.0, cost weight 1.0 and `update: true`;
  - constraint `collision`: ResidualDistanceCollision2 between
    `fer_hand_sc_capsule_0` and `obstacle_capsule_0`, lower 0.01, upper inf.
- **Terminal stage:** the same, without `control_reg`; `state_reg` has cost
  weight 0.0.

The test robot is example-robot-data's panda, which does not have these
geometry names. The test version uses a capsule from panda's collision model
(after `collision_as_capsule`) and an obstacle capsule from a small
environment URDF in `CPP/test/resources/`. The golden generator uses the
same files.

The Python originals of the other test OCPs are in this repo
(`PY/ocp/ocp_goal_reaching.yaml`, and
`agimus_controller_examples/.../panda_pick_and_place/config/ocp_definition_file.yaml`).
