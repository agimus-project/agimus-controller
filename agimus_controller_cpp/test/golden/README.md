# Golden files

Fixtures for the C++ parity tests (design.md, "Golden files"). Each file
records what the Python package `agimus_controller` does for fixed inputs. A
C++ test rebuilds the same problem from the file and compares.

## Regenerate

From the repository root:

```sh
nix develop .#py-judge -c python agimus_controller_cpp/test/golden/generate_golden.py
```

The inputs are fixed numbers (no random generator), and `tomli-w` writes the
keys in a fixed order, so a rerun in the same environment gives byte-identical
files. Check with `git diff --exit-code agimus_controller_cpp/test/golden`
after a rerun. A new pinocchio, crocoddyl, mim_solvers or numpy may change
solver outputs in the last digits; `[meta]` says which versions wrote a file.

The generator never edits the Python package. Where it has to work around a
Python defect to produce a set, the file's `meta.notes` and this README say so.

## Conventions

- Every file starts with `[meta]`: `set`, `python_under_test`,
  `agimus_controller_commit` (last commit touching
  `agimus_controller/agimus_controller`, with ` (dirty)` if it had local
  changes), the versions of pinocchio, crocoddyl, mim_solvers, numpy,
  example-robot-data and Python, `generator`, and `notes`.
- Vectors are arrays of floats. Matrices are arrays of rows. A list of
  matrices (e.g. one gain per tick) is an array of matrices.
- Floats are written with Python's `repr`, which round-trips: read them back
  as `double` and they are bit-identical to Python's values.
- Times on the solver side are `double` seconds (`dt`, `timesteps`,
  `total_time`); time stamps are integer nanoseconds (`*_ns`).
- A state `x` is `[q, v]`.
- Robot `[robot]` (G3, G4, G5, G10a): example-robot-data `panda`
  (`example_robot_data_urdf`/`_srdf` are relative to example-robot-data's
  `share/example-robot-data`), no free flyer, `moving_joint_names` = the 7 arm
  joints, the fingers locked at `q0` = 0, armature 0.1 on every arm joint,
  `collision_as_capsule = true`, `self_collision = false` (as the Python
  tests). End-effector frame: `panda_hand_tcp`.
- `[ocp_params]` (G3, G4, G5, G10a) holds every field of `OCPParamsBaseCroco`
  as used, including the derived `n_controls`, `timesteps`, `total_time`.
  `max_solve_time` is not set (Python `None`) in any set.

## Files

### `g1_ocp_params.toml`: `OCPParamsBaseCroco`

- `inputs.dt = 0.01`.
- `[[cases]]`, three of them: `factors`/`n_steps` = `[1]/[19]`,
  `[1,2]/[20,10]`, `[1,2,4,8]/[5,5,5,4]`; `horizon_size` = sum of `n_steps`;
  outputs `n_controls`, `timesteps`, `total_time`.
- `timesteps[i] = dt * factor` (one IEEE multiplication) matches C++ exactly.
- `total_time` is Python's built-in `sum`, which since Python 3.12 is a
  compensated (Neumaier) sum for floats: it is NOT the left-to-right sum, and
  it differs from it by 1-2 ulp in all three cases (Python 3.14 here). The C++
  comparison of `total_time` is skipped pending a ruling (T4.1 report).

### `g2_warm_start_reference.toml`: `WarmStartReference.generate`

- Robot: example-robot-data `ur5`, full model (`robex.load("ur5").model`), as
  in `test_warm_start_reference.py`, not panda. `[robot]` gives the URDF
  (`robots/ur_description/urdf/ur5_robot.urdf`), `nq`, `nv`.
- Inputs: `initial_state` (`q`, `v`, `a`) and a 5-point `reference` (`q`, `v`,
  `a`, one row per point).
- Outputs: `x0`, `xs` (5 rows), `us` (4 rows). `xs[0]` is the initial state,
  `xs[i]` the reference point i; `us[0] = rnea(initial q, v, a)`,
  `us[i] = rnea(reference[i])` for 1 ≤ i ≤ 3. `reference[0]` and the
  acceleration of the last point are unused. RNEA ignores the armature.

### `g3_warm_start_shift.toml`: `WarmStartShiftPreviousSolution.generate`

- Mirrors `test_warm_start_shift_previous_reference.py`: `dt = 0.1`,
  `factors = [1, 2]`, `n_steps = [2, 1]` (timesteps 0.1, 0.1, 0.2).
- Inputs: `initial_state` (`q` = -10, `v` = 100, as the test),
  `previous_xs` (4 rows) and `previous_us` (3 rows). The previous solution is
  a crocoddyl rollout from x = 0 of fixed controls through the running models
  of `OCPCrocoGoalReaching` (the test uses random controls). Python's
  previous solution has no gains (`ricatti_gains = []`); size them as zeros in
  C++. The reference is ignored by Python (the test passes `[]`).
- Outputs: `x0`, `xs` (4 rows), `us` (3 rows) after one `generate`.
- Pending audit A13 (B18 proposed): the terminal state is never shifted when
  its step is longer than dt, so `outputs.xs[3] == previous_xs[3]`. If B18 is
  accepted, C++ differs at `xs[3]` only.

### `g4_ocp_croco_base.toml`: `test_ocp_croco_base.py` and its pickle

- `[expected]`: `tests/resources/simple_ocp_croco_results.pkl` converted as
  is: `states` (10 rows), `ricatti_gains` (9 matrices 7×14),
  `feed_forward_terms` (9 rows). The Python test compares with
  `assert_array_almost_equal` (decimal 6).
- `[recomputed]`: the same OCP solved in the judge environment, with
  `problem_solved`, `iterations`, `kkt` and `max_abs_diff_to_pkl` (1.6e-11
  when written).
- `[problem]` describes the OCP of the test (`TestOCPCroco`): cost names,
  residual types and weights; target `SE3(I, [1, 1, 1])` on `panda_hand_tcp`;
  state reference = neutral, zero velocity; default quadratic activations.
  Note `running_dt = 0.001`: the test builds
  `IntegratedActionModelEuler(dam)` without a step, so crocoddyl's default is
  used, not `ocp_params.dt`. `terminal_dt = 0`. All running nodes share one
  model object.
- `[inputs]`: `x0`; the warm start is `warm_start_x` repeated
  `n_controls + 1` times and `warm_start_u` repeated `n_controls` times.
  `solver_iters = 100`.

### `g5_ocp_goal_reaching.toml`: `OCPCrocoGoalReaching`

- Mirrors `test_ocp_croco_goal_reaching.py`: `dt = 0.05`, horizon 200,
  `solver_iters = 100`.
- Inputs: `x0` (neutral, zero velocity); warm start `warm_start_x` repeated
  201 times and `warm_start_u` (zeros) repeated 200 times; `reference`: one
  weighted point repeated `n_points = 201` times (`q`, `v`, `tau`, `ee_pose`
  with `translation` and `rotation`, weights `w_q`, `w_v`, `w_tau`, `w_ee`).
- Outputs: `problem_solved`, `iterations`, `qp_iterations`, `kkt`, `xs`
  (201 rows), `us` (200 rows), `K0` (7×14).
- Pending audits: A4 (B12 proposed; the terminal cost takes the frame id of
  the last running node) has no effect here since every point uses the same
  frame. A6 (B14 proposed; state weights sized nq + nv, not ndx) has no
  effect since panda has nq == nv.

### `g10a_mpc_goal_reaching.toml`: `MPC.run` closed loop, 20 ticks

- OCP `OCPCrocoGoalReaching`, `dt = 0.01`, `factors = [1]`, `n_steps = [20]`,
  `solver_iters = 10`; warm start `WarmStartShiftPreviousSolution`;
  `TrajectoryBuffer`.
- `inputs.reference`: `n_points = 40` = `n_ticks + n_controls`, all in the
  buffer before start-up. Every point has `q` = the "ready" configuration,
  `v = a = tau = 0`, the same weights (`w_q`, `w_v`, `w_tau`, `w_ee`) and
  `ee_rotation`; `ee_translation` has one row per point (the target moves
  5 mm along y per point). Point k has `id = k` and `time_ns = k * dt_ns`.
- `inputs.initial_state`: `q`, `v` (0), `a` (0), `time_ns` (0).
- `[startup]` (setup, before tick 0), as the ROS node's
  `setup_mpc_initial_guess`: set the references of the first horizon,
  `WarmStartReference.generate` from the initial state, `OCPBaseCroco.solve`
  with `use_iteration_limits_and_timeout=False` (up to 1000 iterations),
  `WarmStartShiftPreviousSolution.update_previous_solution`. No `clear_past`,
  so tick 0 uses the same horizon. Stored: `problem_solved`, `iterations`,
  `kkt`, `xs`, `us`.
- `[ticks]`, one entry per tick in each array:
  - `time_ns`, `q`, `v`: the state given to `MPC.run`;
  - `reference_id`, `problem_solved`, `nb_iter`, `kkt`: from the debug data;
  - `u0` = `us[0]`, `K0` = `K[0]`, `x1` = `xs[1]` of the solution;
  - `q_next`, `v_next`: the state after the step, which is the next tick's
    `q`, `v`.
- How the state advances: `MPC.integrate(state, u0)`, i.e. one
  `calc` of `ocp.problem.runningModels[0]` (Euler, step `timesteps[0] = dt`)
  at `[q, v]` with control `u0` (the feedback term is zero, since the state
  is the solve's x0). This is what `Mpc::integrate` ports.
- Pending audits and behaviour changes:
  - B1: Python adds `int(dt * 1e-9) = 0` to `time_ns`, so `ticks.time_ns`
    stays 0; C++ adds 10 ms per tick. Time does not affect the outputs
    (`MPC.run` ignores `current_time_ns`, audit A3).
  - B2 is not reached: the buffer always holds at least `n_controls + 1`
    points, and with `factors = [1]` Python's size check agrees with
    `horizon`. No B1/B2 fix is applied in the script.
  - A12 does not apply: `timesteps[0] == dt` (the shift warm start asserts
    it). A13 does not apply: every factor is 1.
- Script-side fix, aliasing: `OCPBaseCroco.integrate` returns
  `runningDatas[0].xnext`, a view into crocoddyl's data, and `MPC.integrate`
  stores slices of it as the state's `q` and `v`. The next solve overwrites
  that data, which would change the state before the following
  `MPC.integrate`. The script copies `q` and `v` right after
  `MPC.integrate`. Without the copy, the closed loop would integrate from
  the solver's last node-0 prediction instead of the state given to `run`.
