#!/usr/bin/env python3
"""Generate the golden TOML files of the C++ parity tests (design.md, Golden files).

Run from the repository root, in the Python judge environment:

    nix develop .#py-judge -c python agimus_controller_cpp/test/golden/generate_golden.py

Every input is a fixed number (no randomness), and the files are written with
a stable key order, so a rerun in the same environment gives byte-identical
files. The script records what the Python package does; it never edits it.
See README.md in this directory for the schema of each file.
"""

from __future__ import annotations

import os
import pickle
import platform
import re
import subprocess
import sys
from pathlib import Path

import crocoddyl
import example_robot_data as robex
import mim_solvers
import numpy as np
import pinocchio as pin
import tomli_w

from agimus_controller.factory.robot_model import RobotModelParameters, RobotModels
from agimus_controller.mpc import MPC
from agimus_controller.mpc_data import OCPResults
from agimus_controller.ocp.ocp_croco_goal_reaching import OCPCrocoGoalReaching
from agimus_controller.ocp_base_croco import OCPBaseCroco
from agimus_controller.ocp_param_base import DTFactorsNSeq, OCPParamsBaseCroco
from agimus_controller.trajectory import (
    TrajectoryBuffer,
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller.warm_start_reference import WarmStartReference
from agimus_controller.warm_start_shift_previous_solution import (
    WarmStartShiftPreviousSolution,
)

GOLDEN_DIR = Path(__file__).resolve().parent
REPO_ROOT = GOLDEN_DIR.parents[2]
PY_PACKAGE = REPO_ROOT / "agimus_controller" / "agimus_controller"
PKL_PATH = (
    REPO_ROOT
    / "agimus_controller"
    / "tests"
    / "resources"
    / "simple_ocp_croco_results.pkl"
)

# Robot of design.md, Golden files: panda, 7 arm joints, armature 0.1.
PANDA_URDF = "robots/panda_description/urdf/panda.urdf"
PANDA_SRDF = "robots/panda_description/srdf/panda.srdf"
PANDA_ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
PANDA_ARMATURE = 0.1
EE_FRAME = "panda_hand_tcp"


# --------------------------------------------------------------------------- #
# Conversion helpers: everything becomes plain Python floats/ints/lists.
# --------------------------------------------------------------------------- #
def vec(v) -> list[float]:
    return [float(x) for x in np.asarray(v, dtype=np.float64).reshape(-1)]


def mat(m) -> list[list[float]]:
    a = np.asarray(m, dtype=np.float64)
    assert a.ndim == 2, a.shape
    return [vec(row) for row in a]


def rows(vs) -> list[list[float]]:
    return [vec(v) for v in vs]


def se3_table(placement: pin.SE3) -> dict:
    return {
        "translation": vec(placement.translation),
        "rotation": mat(placement.rotation),
    }


# --------------------------------------------------------------------------- #
# Metadata.
# --------------------------------------------------------------------------- #
def _store_version(module, name: str) -> str:
    """Version from the module, else from its nix store path (`<name>-<ver>`)."""
    version = getattr(module, "__version__", None)
    if version:
        return str(version)
    real = os.path.realpath(module.__file__)
    match = re.search(
        rf"/nix/store/[a-z0-9]+-(?:py-|python[0-9.]+-)?{name}-([0-9][^/]*)/", real
    )
    if match:
        return match.group(1)
    return "unknown"


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(REPO_ROOT), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _py_commit() -> str:
    commit = _git("log", "-1", "--format=%H", "--", str(PY_PACKAGE))
    if _git("status", "--porcelain", "--", str(PY_PACKAGE)):
        commit += " (dirty)"
    return commit


VERSIONS = None  # Filled in main(), shared by every [meta].


def meta(set_id: str, under_test: str, notes: list[str]) -> dict:
    return {
        "set": set_id,
        "python_under_test": under_test,
        "agimus_controller_commit": VERSIONS["commit"],
        "pinocchio": VERSIONS["pinocchio"],
        "crocoddyl": VERSIONS["crocoddyl"],
        "mim_solvers": VERSIONS["mim_solvers"],
        "numpy": VERSIONS["numpy"],
        "example_robot_data": VERSIONS["example_robot_data"],
        "python": VERSIONS["python"],
        "generator": "agimus_controller_cpp/test/golden/generate_golden.py",
        "notes": notes,
    }


def write(name: str, content: dict) -> None:
    path = GOLDEN_DIR / name
    with open(path, "wb") as f:
        tomli_w.dump(content, f)
    print(f"wrote {path.relative_to(REPO_ROOT)}")


# --------------------------------------------------------------------------- #
# Shared builders.
# --------------------------------------------------------------------------- #
def panda_robot_models() -> RobotModels:
    """Panda as in the Python tests: fingers locked at 0, arm free, armature 0.1."""
    robot = robex.load("panda")
    urdf_path = Path(robot.urdf)
    srdf_path = Path(robot.urdf.replace("urdf", "srdf"))
    params = RobotModelParameters(
        q0=np.zeros(robot.model.nq),
        free_flyer=False,
        moving_joint_names=list(PANDA_ARM_JOINTS),
        robot_urdf=urdf_path,
        srdf=srdf_path,
        urdf_meshes_dir=urdf_path.parent.parent.parent.parent.parent,
        collision_as_capsule=True,
        self_collision=False,
        armature=np.full(len(PANDA_ARM_JOINTS), PANDA_ARMATURE),
    )
    return RobotModels(params)


def panda_robot_table(robot_models: RobotModels) -> dict:
    return {
        "example_robot_data_urdf": PANDA_URDF,
        "example_robot_data_srdf": PANDA_SRDF,
        "free_flyer": False,
        "q0": vec(robot_models.params.q0),
        "moving_joint_names": list(PANDA_ARM_JOINTS),
        "armature": vec(robot_models.armature),
        "collision_as_capsule": True,
        "self_collision": False,
        "nq": int(robot_models.robot_model.nq),
        "nv": int(robot_models.robot_model.nv),
    }


def ocp_params_table(p: OCPParamsBaseCroco) -> dict:
    t = {
        "dt": float(p.dt),
        "solver_iters": int(p.solver_iters),
        "factors": [int(f) for f in p.dt_factor_n_seq.factors],
        "n_steps": [int(n) for n in p.dt_factor_n_seq.n_steps],
        "horizon_size": int(p.horizon_size),
        "qp_iters": int(p.qp_iters),
        "termination_tolerance": float(p.termination_tolerance),
        "eps_abs": float(p.eps_abs),
        "eps_rel": float(p.eps_rel),
        "callbacks": bool(p.callbacks),
        "use_debug_data": bool(p.use_debug_data),
        "use_filter_line_search": bool(p.use_filter_line_search),
        "n_threads": int(p.n_threads),
        "n_controls": int(p.n_controls),
        "timesteps": vec(p.timesteps),
        "total_time": float(p.total_time),
    }
    assert p.max_solve_time is None, "max_solve_time is not stored"
    return t


def weighted_point(q, v, a, tau, ee_pose, w_q, w_v, w_tau, w_ee):
    return WeightedTrajectoryPoint(
        TrajectoryPoint(
            robot_configuration=q,
            robot_velocity=v,
            robot_acceleration=a,
            robot_effort=tau,
            end_effector_poses={EE_FRAME: ee_pose},
        ),
        TrajectoryPointWeights(
            w_robot_configuration=w_q,
            w_robot_velocity=w_v,
            w_robot_effort=w_tau,
            w_end_effector_poses={EE_FRAME: w_ee},
        ),
    )


# --------------------------------------------------------------------------- #
# G1: OCPParamsBaseCroco.
# --------------------------------------------------------------------------- #
def g1() -> None:
    dt = 0.01
    cases = []
    for factors, n_steps in (
        ([1], [19]),
        ([1, 2], [20, 10]),
        ([1, 2, 4, 8], [5, 5, 5, 4]),
    ):
        p = OCPParamsBaseCroco(
            dt=dt,
            solver_iters=10,
            dt_factor_n_seq=DTFactorsNSeq(factors=factors, n_steps=n_steps),
            horizon_size=sum(n_steps),
        )
        cases.append(
            {
                "factors": factors,
                "n_steps": n_steps,
                "horizon_size": sum(n_steps),
                "n_controls": int(p.n_controls),
                "timesteps": vec(p.timesteps),
                "total_time": float(p.total_time),
            }
        )
    write(
        "g1_ocp_params.toml",
        {
            "meta": meta(
                "G1",
                "agimus_controller.ocp_param_base.OCPParamsBaseCroco",
                [
                    "horizon_size = sum(n_steps) for each case.",
                    "timesteps[i] = dt * factor (one IEEE multiplication): compare "
                    "exactly.",
                    "total_time = Python's built-in sum(timesteps). Since Python 3.12 "
                    "sum() of floats is compensated (Neumaier), so it is not the "
                    "left-to-right sum; here every case differs from it by 1-2 ulp.",
                ],
            ),
            "inputs": {"dt": dt},
            "cases": cases,
        },
    )


# --------------------------------------------------------------------------- #
# G2: WarmStartReference.generate (mirrors test_warm_start_reference.py: ur5).
# --------------------------------------------------------------------------- #
def g2() -> None:
    robot = robex.load("ur5")
    rmodel = robot.model
    nq, nv = rmodel.nq, rmodel.nv
    n_points = 5
    jq = np.arange(nq, dtype=np.float64)
    jv = np.arange(nv, dtype=np.float64)

    # Fixed, non-trivial values built with exact-enough arithmetic.
    def q_of(k):
        return 0.3 - 0.1 * jq + 0.05 * k - 0.01 * k * jq

    def v_of(k):
        return -0.2 + 0.07 * jv - 0.03 * k + 0.002 * k * jv

    def a_of(k):
        return 0.5 - 0.15 * jv + 0.1 * k + 0.004 * k * jv

    initial = TrajectoryPoint(
        robot_configuration=0.1 + 0.02 * jq,
        robot_velocity=-0.05 + 0.03 * jv,
        robot_acceleration=0.2 - 0.06 * jv,
    )
    reference = [
        TrajectoryPoint(
            robot_configuration=q_of(k),
            robot_velocity=v_of(k),
            robot_acceleration=a_of(k),
        )
        for k in range(n_points)
    ]
    ws = WarmStartReference()
    ws.setup(rmodel)
    x0, xs, us = ws.generate(initial, reference)
    assert len(xs) == n_points and len(us) == n_points - 1

    write(
        "g2_warm_start_reference.toml",
        {
            "meta": meta(
                "G2",
                "agimus_controller.warm_start_reference.WarmStartReference.generate",
                [
                    "Robot: example-robot-data ur5, full model (robex.load('ur5').model), "
                    "as in test_warm_start_reference.py, not panda.",
                    "xs[0] = initial state; xs[i] = reference[i] for i >= 1; "
                    "us[0] = rnea(initial q, v, a); us[i] = rnea(reference[i]) for 1 <= i < n-1. "
                    "reference[0] and the acceleration of reference[n-1] are unused.",
                    "RNEA ignores the armature (WarmStartReference uses pin.rnea on the model only).",
                ],
            ),
            "robot": {
                "example_robot_data_urdf": "robots/ur_description/urdf/ur5_robot.urdf",
                "free_flyer": False,
                "nq": int(nq),
                "nv": int(nv),
            },
            "inputs": {
                "initial_state": {
                    "q": vec(initial.robot_configuration),
                    "v": vec(initial.robot_velocity),
                    "a": vec(initial.robot_acceleration),
                },
                "reference": {
                    "q": rows(p.robot_configuration for p in reference),
                    "v": rows(p.robot_velocity for p in reference),
                    "a": rows(p.robot_acceleration for p in reference),
                },
            },
            "outputs": {"x0": vec(x0), "xs": rows(xs), "us": rows(us)},
        },
    )


# --------------------------------------------------------------------------- #
# G3: WarmStartShiftPreviousSolution.generate (mirrors
# test_warm_start_shift_previous_reference.py, fixed controls).
# --------------------------------------------------------------------------- #
def g3() -> None:
    robot_models = panda_robot_models()
    model = robot_models.robot_model
    nq, nv = model.nq, model.nv
    p = OCPParamsBaseCroco(
        dt=0.1,
        solver_iters=1000,
        horizon_size=3,
        dt_factor_n_seq=DTFactorsNSeq(factors=[1, 2], n_steps=[2, 1]),
    )
    assert p.timesteps == (0.1, 0.1, 0.2)
    ws = WarmStartShiftPreviousSolution()
    ws.setup(robot_models, p)

    # Previous solution: a rollout of fixed controls through the goal-reaching
    # OCP's dynamics, from x = 0 (the test uses random controls).
    ocp = OCPCrocoGoalReaching(robot_models, p)
    j = np.arange(nv, dtype=np.float64)
    controls = [0.5 + 0.1 * j - 0.25 * i + 0.05 * i * j for i in range(p.n_controls)]
    ocp.problem.x0 = np.zeros(nq + nv)
    states = [np.array(x) for x in ocp.problem.rollout(controls)]
    prev_states = [x.copy() for x in states]
    prev_controls = [u.copy() for u in controls]

    ws.update_previous_solution(
        OCPResults(
            states=list(states), ricatti_gains=[], feed_forward_terms=list(controls)
        )
    )
    initial = TrajectoryPoint(
        robot_configuration=-10 * np.ones(nq),
        robot_velocity=100 * np.ones(nv),
    )
    x0, xs, us = ws.generate(initial, [])

    write(
        "g3_warm_start_shift.toml",
        {
            "meta": meta(
                "G3",
                "agimus_controller.warm_start_shift_previous_solution."
                "WarmStartShiftPreviousSolution.generate",
                [
                    "Python generate ignores the reference; the test passes []. "
                    "Any horizon may be given to the C++ generate.",
                    "Previous solution = crocoddyl rollout of inputs.previous_us from x = 0 "
                    "through OCPCrocoGoalReaching's running models; ricatti_gains empty "
                    "in Python (size them as zeros in C++).",
                    "Pending audit A13 (B18 proposed): the terminal state xs[N] is never "
                    "shifted when its step > dt; outputs.xs[N] equals previous_xs[N]. "
                    "If B18 is accepted, C++ differs at xs[N] only.",
                    "The shift integrator is Euler over free forward dynamics with "
                    "armature, step dt = timesteps[0].",
                ],
            ),
            "robot": panda_robot_table(robot_models),
            "ocp_params": ocp_params_table(p),
            "inputs": {
                "initial_state": {
                    "q": vec(initial.robot_configuration),
                    "v": vec(initial.robot_velocity),
                },
                "previous_xs": rows(prev_states),
                "previous_us": rows(prev_controls),
            },
            "outputs": {"x0": vec(x0), "xs": rows(xs), "us": rows(us)},
        },
    )


# --------------------------------------------------------------------------- #
# G4: test_ocp_croco_base.py and its pickle.
# --------------------------------------------------------------------------- #
class _TestOCPCroco(OCPBaseCroco):
    """Copy of TestSimpleOCPCroco.TestOCPCroco (PYT/test_ocp_croco_base.py)."""

    def create_running_model_list(self):
        running_cost_model = crocoddyl.CostModelSum(self._state)
        x_residual = crocoddyl.ResidualModelState(
            self._state,
            np.concatenate(
                (
                    pin.neutral(self._robot_models.robot_model),
                    np.zeros(self._robot_models.robot_model.nv),
                )
            ),
        )
        x_reg_cost = crocoddyl.CostModelResidual(self._state, x_residual)
        u_residual = crocoddyl.ResidualModelControl(self._state)
        u_reg_cost = crocoddyl.CostModelResidual(self._state, u_residual)
        frame_placement_residual = crocoddyl.ResidualModelFramePlacement(
            self._state,
            self._robot_models.robot_model.getFrameId("panda_hand_tcp"),
            pin.SE3(np.eye(3), np.array([1.0, 1.0, 1.0])),
        )
        goal_tracking_cost = crocoddyl.CostModelResidual(
            self._state, frame_placement_residual
        )
        running_cost_model.addCost("stateReg", x_reg_cost, 0.1)
        running_cost_model.addCost("ctrlRegGrav", u_reg_cost, 0.0001)
        running_cost_model.addCost("gripperPoseRM", goal_tracking_cost, 1.0)
        running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state, self._actuation, running_cost_model
        )
        running_model = crocoddyl.IntegratedActionModelEuler(running_DAM)
        running_model.differential.armature = self._robot_models.armature
        return [running_model] * self._ocp_params.n_controls

    def create_terminal_model(self):
        terminal_cost_model = crocoddyl.CostModelSum(self._state)
        x_residual = crocoddyl.ResidualModelState(
            self._state,
            np.concatenate(
                (
                    pin.neutral(self._robot_models.robot_model),
                    np.zeros(self._robot_models.robot_model.nv),
                )
            ),
        )
        x_reg_cost = crocoddyl.CostModelResidual(self._state, x_residual)
        frame_placement_residual = crocoddyl.ResidualModelFramePlacement(
            self._state,
            self._robot_models.robot_model.getFrameId("panda_hand_tcp"),
            pin.SE3(np.eye(3), np.array([1.0, 1.0, 1.0])),
        )
        goal_tracking_cost = crocoddyl.CostModelResidual(
            self._state, frame_placement_residual
        )
        terminal_cost_model.addCost("stateReg", x_reg_cost, 0.1)
        terminal_cost_model.addCost("gripperPose", goal_tracking_cost, 50)
        terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state, self._actuation, terminal_cost_model
        )
        terminal_model = crocoddyl.IntegratedActionModelEuler(terminal_DAM, 0.0)
        terminal_model.differential.armature = self._robot_models.armature
        return terminal_model

    def set_reference_weighted_trajectory(self, reference_weighted_trajectory):
        return None


def g4() -> None:
    robot_models = panda_robot_models()
    model = robot_models.robot_model
    p = OCPParamsBaseCroco(
        dt=0.1,
        horizon_size=9,
        dt_factor_n_seq=DTFactorsNSeq(factors=[1], n_steps=[9]),
        solver_iters=100,
        callbacks=False,
    )
    x0 = np.concatenate((pin.neutral(model), np.zeros(model.nv)))
    xs_ws = [np.zeros(model.nq + model.nv)] * (p.n_controls + 1)
    us_ws = [np.zeros(model.nq)] * p.n_controls
    ocp = _TestOCPCroco(robot_models, p)
    ocp.solve(x0, xs_ws, us_ws)
    running_dt = float(ocp.problem.runningModels[0].dt)
    terminal_dt = float(ocp.problem.terminalModel.dt)

    with open(PKL_PATH, "rb") as f:
        pkl = pickle.load(f)
    pkl_xs = np.array(pkl["states"])
    pkl_K = np.array(pkl["ricatti_gains"])
    pkl_us = np.array(pkl["feed_forward_terms"])
    res = ocp.ocp_results
    xs = np.array(res.states.tolist())
    K = np.array(res.ricatti_gains.tolist())
    us = np.array(res.feed_forward_terms.tolist())
    max_diff = max(
        float(np.max(np.abs(xs - pkl_xs))),
        float(np.max(np.abs(K - pkl_K))),
        float(np.max(np.abs(us - pkl_us))),
    )
    dbg = ocp.debug_data

    write(
        "g4_ocp_croco_base.toml",
        {
            "meta": meta(
                "G4",
                "agimus_controller.ocp_base_croco.OCPBaseCroco "
                "(TestSimpleOCPCroco of test_ocp_croco_base.py)",
                [
                    "[expected] is resources/simple_ocp_croco_results.pkl, converted as is; "
                    "the Python test compares it with assert_array_almost_equal (decimal 6).",
                    "[recomputed] is the same OCP solved by this environment; "
                    "recomputed.max_abs_diff_to_pkl is its largest difference to [expected].",
                    "The running models use crocoddyl's default Euler step "
                    "(problem.running_dt = 1e-3), NOT ocp_params.dt: the test builds "
                    "IntegratedActionModelEuler(dam) without a step.",
                    "All running nodes share one model object ([model] * n_controls).",
                    "Costs use the default quadratic activation (CostModelResidual without "
                    "activation); ctrlRegGrav is a ResidualModelControl (reference 0), "
                    "not ControlGrav, despite its name.",
                ],
            ),
            "robot": panda_robot_table(robot_models),
            "ocp_params": ocp_params_table(p),
            "problem": {
                "ee_frame": EE_FRAME,
                "running_dt": running_dt,
                "terminal_dt": terminal_dt,
                "state_reference": vec(x0),
                "ee_target": se3_table(pin.SE3(np.eye(3), np.array([1.0, 1.0, 1.0]))),
                "running_costs": {
                    "stateReg": {"residual": "State", "weight": 0.1},
                    "ctrlRegGrav": {"residual": "Control", "weight": 0.0001},
                    "gripperPoseRM": {"residual": "FramePlacement", "weight": 1.0},
                },
                "terminal_costs": {
                    "stateReg": {"residual": "State", "weight": 0.1},
                    "gripperPose": {"residual": "FramePlacement", "weight": 50.0},
                },
            },
            "inputs": {
                "x0": vec(x0),
                "warm_start_x": vec(xs_ws[0]),
                "warm_start_u": vec(us_ws[0]),
                "warm_start_note": "warm_start_x repeated n_controls + 1 times, "
                "warm_start_u repeated n_controls times",
            },
            "expected": {
                "states": rows(pkl_xs),
                "ricatti_gains": [mat(k) for k in pkl_K],
                "feed_forward_terms": rows(pkl_us),
            },
            "recomputed": {
                "max_abs_diff_to_pkl": max_diff,
                "problem_solved": bool(dbg.problem_solved),
                "iterations": int(dbg.nb_iter),
                "kkt": float(dbg.kkt_norm),
                "states": rows(xs),
                "ricatti_gains": [mat(k) for k in K],
                "feed_forward_terms": rows(us),
            },
        },
    )


# --------------------------------------------------------------------------- #
# G5: OCPCrocoGoalReaching (mirrors test_ocp_croco_goal_reaching.py).
# --------------------------------------------------------------------------- #
def g5() -> None:
    robot_models = panda_robot_models()
    model = robot_models.robot_model
    nq, nv = model.nq, model.nv
    p = OCPParamsBaseCroco(
        dt=0.05,
        horizon_size=200,
        dt_factor_n_seq=DTFactorsNSeq(factors=[1], n_steps=[200]),
        solver_iters=100,
        callbacks=False,
    )
    q0 = pin.neutral(model)
    ee_pose = pin.SE3(np.eye(3), np.array([0.5, 0.2, 0.5]))
    n_states = p.n_controls + 1
    x0 = np.concatenate((q0, np.zeros(nv)))
    xs_ws = [x0] * n_states
    us_ws = [np.zeros(nv)] * p.n_controls
    point = weighted_point(
        q=q0,
        v=np.zeros(nv),
        a=None,
        tau=np.zeros(nv),
        ee_pose=ee_pose,
        w_q=0.01 * np.ones(nq),
        w_v=0.01 * np.ones(nv),
        w_tau=0.0001 * np.ones(nv),
        w_ee=1e3 * np.ones(6),
    )
    ocp = OCPCrocoGoalReaching(robot_models, p)
    ocp.set_reference_weighted_trajectory([point] * n_states)
    ocp.solve(x0, xs_ws, us_ws)
    res = ocp.ocp_results
    dbg = ocp.debug_data

    write(
        "g5_ocp_goal_reaching.toml",
        {
            "meta": meta(
                "G5",
                "agimus_controller.ocp.ocp_croco_goal_reaching.OCPCrocoGoalReaching",
                [
                    "The reference is one weighted point repeated n_points = n_controls + 1 "
                    "times; the warm start is x0 repeated n_controls + 1 times and zero "
                    "controls.",
                    "Pending audit A4 (B12 proposed): the terminal goalTracking cost uses "
                    "the ee frame id of the last running node; same frame here, no effect.",
                    "Pending audit A6 (B14 proposed): state weights sized nq + nv instead of "
                    "ndx; panda has nq == nv, no effect.",
                    "Running costs: stateReg (State, WeightedQuad), ctrlReg (Control, "
                    "WeightedQuad), goalTracking (FramePlacement, WeightedQuad), each with "
                    "cost weight 1; terminal: stateReg and goalTracking. Terminal Euler "
                    "step 0.",
                ],
            ),
            "robot": panda_robot_table(robot_models),
            "ocp_params": ocp_params_table(p),
            "inputs": {
                "x0": vec(x0),
                "warm_start_x": vec(x0),
                "warm_start_u": vec(np.zeros(nv)),
                "reference": {
                    "n_points": n_states,
                    "ee_frame": EE_FRAME,
                    "q": vec(point.point.robot_configuration),
                    "v": vec(point.point.robot_velocity),
                    "tau": vec(point.point.robot_effort),
                    "ee_pose": se3_table(ee_pose),
                    "w_q": vec(point.weights.w_robot_configuration),
                    "w_v": vec(point.weights.w_robot_velocity),
                    "w_tau": vec(point.weights.w_robot_effort),
                    "w_ee": vec(point.weights.w_end_effector_poses[EE_FRAME]),
                },
            },
            "outputs": {
                "problem_solved": bool(dbg.problem_solved),
                "iterations": int(dbg.nb_iter),
                "qp_iterations": int(dbg.nb_qp_iter),
                "kkt": float(dbg.kkt_norm),
                "xs": rows(res.states),
                "us": rows(res.feed_forward_terms),
                "K0": mat(res.ricatti_gains[0]),
            },
        },
    )


# --------------------------------------------------------------------------- #
# G10a: MPC.run closed loop, goal-reaching OCP, 20 ticks.
# --------------------------------------------------------------------------- #
def g10a() -> None:
    robot_models = panda_robot_models()
    model = robot_models.robot_model
    nq, nv = model.nq, model.nv
    n_ticks = 20
    p = OCPParamsBaseCroco(
        dt=0.01,
        horizon_size=20,
        dt_factor_n_seq=DTFactorsNSeq(factors=[1], n_steps=[20]),
        solver_iters=10,
        callbacks=False,
    )
    dt_ns = 10_000_000

    # Reference: arm at the "ready" configuration, the hand target moving
    # along y by 5 mm per point, pointing down.
    q_ref = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    rot_down = np.diag([1.0, -1.0, -1.0])
    n_points = n_ticks + p.n_controls  # tick k uses points k .. k + n_controls.
    targets = [np.array([0.45, -0.05 + 0.005 * k, 0.45]) for k in range(n_points)]
    w_q = 0.01 * np.ones(nq)
    w_v = 0.01 * np.ones(nv)
    w_tau = 0.0001 * np.ones(nv)
    w_ee = 1e3 * np.ones(6)
    buffer = TrajectoryBuffer(p.dt_factor_n_seq)
    for k in range(n_points):
        wp = weighted_point(
            q=q_ref.copy(),
            v=np.zeros(nv),
            a=np.zeros(nv),
            tau=np.zeros(nv),
            ee_pose=pin.SE3(rot_down, targets[k]),
            w_q=w_q,
            w_v=w_v,
            w_tau=w_tau,
            w_ee=w_ee,
        )
        wp.point.id = k
        wp.point.time_ns = k * dt_ns
        buffer.append(wp)

    ocp = OCPCrocoGoalReaching(robot_models, p)
    ws_shift = WarmStartShiftPreviousSolution()
    ws_shift.setup(robot_models, p)
    mpc = MPC()
    mpc.setup(ocp=ocp, warm_start=ws_shift, buffer=buffer)

    q_init = np.array([0.1, -0.7, -0.1, -2.2, 0.05, 1.5, 0.7])
    v_init = np.zeros(nv)
    state = TrajectoryPoint(
        time_ns=0,
        robot_configuration=q_init.copy(),
        robot_velocity=v_init.copy(),
        robot_acceleration=np.zeros(nv),
    )

    # Start-up, as ROS setup_mpc_initial_guess (agimus_controller_ros/
    # agimus_controller.py:368-390): references of the first horizon,
    # WarmStartReference, solve without limits, seed the shift warm start.
    # No clear_past: tick 0 sees the same horizon.
    ws_ref = WarmStartReference()
    ws_ref.setup(model)
    horizon = buffer.horizon
    ocp.set_reference_weighted_trajectory(horizon)
    x0, x_init, u_init = ws_ref.generate(state, [el.point for el in horizon])
    ocp.solve(x0, x_init, u_init, use_iteration_limits_and_timeout=False)
    ws_shift.update_previous_solution(ocp.ocp_results)
    startup = {
        "problem_solved": bool(ocp.debug_data.problem_solved),
        "iterations": int(ocp.debug_data.nb_iter),
        "kkt": float(ocp.debug_data.kkt_norm),
        "xs": rows(ocp.ocp_results.states),
        "us": rows(ocp.ocp_results.feed_forward_terms),
    }

    ticks = {
        k: []
        for k in (
            "time_ns",
            "q",
            "v",
            "reference_id",
            "problem_solved",
            "nb_iter",
            "kkt",
            "u0",
            "K0",
            "x1",
            "q_next",
            "v_next",
        )
    }
    for _ in range(n_ticks):
        ticks["time_ns"].append(int(state.time_ns))
        ticks["q"].append(vec(state.robot_configuration))
        ticks["v"].append(vec(state.robot_velocity))
        res = mpc.run(state, state.time_ns)
        assert res is not None
        dbg = mpc.mpc_debug_data
        u0 = np.array(res.feed_forward_terms[0])
        ticks["reference_id"].append(int(dbg.reference_id))
        ticks["problem_solved"].append(bool(dbg.ocp.problem_solved))
        ticks["nb_iter"].append(int(dbg.ocp.nb_iter))
        ticks["kkt"].append(float(dbg.ocp.kkt_norm))
        ticks["u0"].append(vec(u0))
        ticks["K0"].append(mat(res.ricatti_gains[0]))
        ticks["x1"].append(vec(res.states[1]))

        # Advance the state with Python's MPC.integrate. Its q and v are views
        # of ocp.problem.runningDatas[0].xnext, which the next solve
        # overwrites: copy them at once (see README, aliasing).
        state = TrajectoryPoint(
            time_ns=state.time_ns,
            robot_configuration=np.array(ticks["q"][-1]),
            robot_velocity=np.array(ticks["v"][-1]),
            robot_acceleration=np.zeros(nv),
        )
        state = mpc.integrate(state, u0)
        state.robot_configuration = np.array(state.robot_configuration, copy=True)
        state.robot_velocity = np.array(state.robot_velocity, copy=True)
        ticks["q_next"].append(vec(state.robot_configuration))
        ticks["v_next"].append(vec(state.robot_velocity))

    write(
        "g10a_mpc_goal_reaching.toml",
        {
            "meta": meta(
                "G10a",
                "agimus_controller.mpc.MPC.run with OCPCrocoGoalReaching and "
                "WarmStartShiftPreviousSolution",
                [
                    "No B1/B2 fix applied: the loop runs without them. The buffer holds "
                    "n_ticks + n_controls points from the start, so the Python size check "
                    "(B2) never fails, and factors = [1] makes it agree with horizon().",
                    "Start-up (setup, before tick 0) mirrors the ROS node: set the "
                    "references of the first horizon, WarmStartReference.generate from the "
                    "initial state (a = 0), OCPBaseCroco.solve with "
                    "use_iteration_limits_and_timeout=False (max 1000 iterations), then "
                    "WarmStartShiftPreviousSolution.update_previous_solution. No clear_past.",
                    "Each tick: MPC.run(state) (references, shift warm start, solve with "
                    "solver_iters, update_previous_solution, clear_past); then the state is "
                    "advanced with MPC.integrate(state, u0), u0 = us[0] of the solution "
                    "(the feedback term is 0 since the state is x0): Euler step of "
                    "ocp.problem.runningModels[0] (step timesteps[0] = dt) on [q, v].",
                    "B1: Python's MPC.integrate adds int(dt * 1e-9) = 0 to time_ns, so "
                    "ticks.time_ns stays 0. C++ adds dt (10 ms). Time does not influence "
                    "the solve (MPC.run ignores current_time_ns, audit A3).",
                    "A12 does not apply: timesteps[0] == dt (the shift warm start requires "
                    "it).",
                    "A13 does not apply: every factor is 1.",
                    "Aliasing fix in the script: MPC.integrate returns views of "
                    "runningDatas[0].xnext; q and v are copied right after integrate, "
                    "otherwise the next solve would overwrite the state before it is "
                    "integrated.",
                ],
            ),
            "robot": panda_robot_table(robot_models),
            "ocp_params": ocp_params_table(p),
            "inputs": {
                "n_ticks": n_ticks,
                "dt_ns": dt_ns,
                "initial_state": {
                    "time_ns": 0,
                    "q": vec(q_init),
                    "v": vec(v_init),
                    "a": vec(np.zeros(nv)),
                },
                "reference": {
                    "n_points": n_points,
                    "ee_frame": EE_FRAME,
                    "id": list(range(n_points)),
                    "time_ns": [k * dt_ns for k in range(n_points)],
                    "q": vec(q_ref),
                    "v": vec(np.zeros(nv)),
                    "a": vec(np.zeros(nv)),
                    "tau": vec(np.zeros(nv)),
                    "ee_rotation": mat(rot_down),
                    "ee_translation": rows(targets),
                    "w_q": vec(w_q),
                    "w_v": vec(w_v),
                    "w_tau": vec(w_tau),
                    "w_ee": vec(w_ee),
                },
            },
            "startup": startup,
            "ticks": ticks,
        },
    )


def main() -> None:
    global VERSIONS
    VERSIONS = {
        "commit": _py_commit(),
        "pinocchio": str(pin.__version__),
        "crocoddyl": str(crocoddyl.__version__),
        "mim_solvers": _store_version(mim_solvers, "mim-solvers"),
        "numpy": str(np.__version__),
        "example_robot_data": _store_version(robex, "example-robot-data"),
        "python": platform.python_version(),
    }
    print(VERSIONS, file=sys.stderr)
    g1()
    g2()
    g3()
    g4()
    g5()
    g10a()


if __name__ == "__main__":
    main()
