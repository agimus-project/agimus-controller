# OCP files in agimus_controller_cpp

How agimus uses crocoddyl-toml ([crocoddyl_toml.md](crocoddyl_toml.md)): the
types agimus registers, the `[agimus]` table, and how the horizon is built
from the file's models. Everything agimus-specific lives here, so that
nothing agimus-specific leaks into crocoddyl-toml (a loco-3d library). Briefs
for `CT/` tasks do not include this file.

An OCP is one TOML file: crocoddyl-toml's `models` (and optionally
`problem`), plus an `[agimus]` application table that crocoddyl-toml passes
through untouched (`Document::extra`). For the goal-reaching OCP of
crocoddyl_toml.md's example, the agimus part is:

```toml
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

## Types registered by agimus_controller_cpp

Each is a schema file in `CPP/schemas/toml/<family>/<type>.schema.json` and
a build function in `CPP/src/toml_types/`, added with crocoddyl-toml's CMake
function `crocoddyl_toml_add_types` (crocoddyl_toml.md, Schema files), which
generates the parsing and the idempotent registration function
`register_agimus_controller_cpp_types()`. Registration must happen before a
file is loaded, because loading validates the registered types' keys. So
agimus loads OCP files only through `agimus_controller::load_ocp_file(path)`
(setup), which:

1. calls the generated registration function (idempotent), so loading
   several files never registers a name twice;
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
