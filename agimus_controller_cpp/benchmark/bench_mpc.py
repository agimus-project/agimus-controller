#!/usr/bin/env python3
"""Python MPC.run on the problem of benchmark/bench_mpc.cpp (golden set G10a).

Run from the repository root, in the Python judge environment:

    nix develop .#py-judge -c python agimus_controller_cpp/benchmark/bench_mpc.py

The setup is generate_golden.py's G10a (imported, not copied): OCPCrocoGoalReaching
on the panda, WarmStartShiftPreviousSolution, a sliding buffer, the start-up of
the ROS node. The reference is G10a's, bouncing after point 40 as in
bench_mpc.cpp. Each tick is MPC.run closed by MPC.integrate, timed with
time.perf_counter_ns; the phase timers are Python's own (MPCDebugData).
Options: --ticks N, --warmup N.
"""

from __future__ import annotations

import argparse
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "test" / "golden"))
import generate_golden as gg  # noqa: E402

from agimus_controller.mpc import MPC  # noqa: E402
from agimus_controller.ocp.ocp_croco_goal_reaching import (  # noqa: E402
    OCPCrocoGoalReaching,
)
from agimus_controller.ocp_param_base import (  # noqa: E402
    DTFactorsNSeq,
    OCPParamsBaseCroco,
)
from agimus_controller.trajectory import TrajectoryBuffer, TrajectoryPoint  # noqa: E402
from agimus_controller.warm_start_reference import WarmStartReference  # noqa: E402
from agimus_controller.warm_start_shift_previous_solution import (  # noqa: E402
    WarmStartShiftPreviousSolution,
)

HALF_PERIOD = 40  # points, see bench_mpc.cpp


def first_line(path: str) -> str:
    try:
        return Path(path).read_text().splitlines()[0]
    except (OSError, IndexError):
        return "n/a"


def cpu_model() -> str:
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name"):
            return line.split(":", 1)[1].strip()
    return "n/a"


def percentile_us(values_ns: list[int], q: float) -> float:
    v = sorted(values_ns)
    rank = max(int(-(-q * len(v) // 1)), 1)  # ceil
    return v[rank - 1] / 1e3


def median_us(values_ns: list[int]) -> float:
    return statistics.median(values_ns) / 1e3


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=1000)
    ap.add_argument("--warmup", type=int, default=50)
    args = ap.parse_args()

    robot_models = gg.panda_robot_models()
    model = robot_models.robot_model
    nq, nv = model.nq, model.nv
    p = OCPParamsBaseCroco(
        dt=0.01,
        horizon_size=20,
        dt_factor_n_seq=DTFactorsNSeq(factors=[1], n_steps=[20]),
        solver_iters=10,
        callbacks=False,
        n_threads=1,
    )
    dt_ns = 10_000_000
    n_initial = 2 * (p.n_controls + 1)
    total = args.warmup + args.ticks

    q_ref = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    rot_down = np.diag([1.0, -1.0, -1.0])

    def point(k: int):
        m = k % (2 * HALF_PERIOD)
        steps = m if m <= HALF_PERIOD else 2 * HALF_PERIOD - m
        wp = gg.weighted_point(
            q=q_ref.copy(),
            v=np.zeros(nv),
            a=np.zeros(nv),
            tau=np.zeros(nv),
            ee_pose=pin.SE3(rot_down, np.array([0.45, -0.05 + 0.005 * steps, 0.45])),
            w_q=0.01 * np.ones(nq),
            w_v=0.01 * np.ones(nv),
            w_tau=0.0001 * np.ones(nv),
            w_ee=1e3 * np.ones(6),
        )
        wp.point.id = k
        wp.point.time_ns = k * dt_ns
        return wp

    buffer = TrajectoryBuffer(p.dt_factor_n_seq)
    for k in range(n_initial):
        buffer.append(point(k))
    ocp = OCPCrocoGoalReaching(robot_models, p)
    ws_shift = WarmStartShiftPreviousSolution()
    ws_shift.setup(robot_models, p)
    mpc = MPC()
    mpc.setup(ocp=ocp, warm_start=ws_shift, buffer=buffer)

    state = TrajectoryPoint(
        time_ns=0,
        robot_configuration=np.array([0.1, -0.7, -0.1, -2.2, 0.05, 1.5, 0.7]),
        robot_velocity=np.zeros(nv),
        robot_acceleration=np.zeros(nv),
    )
    # Start-up as the ROS node (generate_golden.py, g10a).
    ws_ref = WarmStartReference()
    ws_ref.setup(model)
    horizon = buffer.horizon
    ocp.set_reference_weighted_trajectory(horizon)
    x0, x_init, u_init = ws_ref.generate(state, [el.point for el in horizon])
    ocp.solve(x0, x_init, u_init, use_iteration_limits_and_timeout=False)
    ws_shift.update_previous_solution(ocp.ocp_results)

    tick, refs, warm, solve, iters, qp_iters = [], [], [], [], [], []
    for k in range(total):
        t0 = time.perf_counter_ns()
        res = mpc.run(state, state.time_ns)
        assert res is not None
        u0 = np.array(res.feed_forward_terms[0])
        state = mpc.integrate(state, u0)
        # Python's integrate returns views of the solver's data: copy them
        # (part of the tick).
        state.robot_configuration = np.array(state.robot_configuration, copy=True)
        state.robot_velocity = np.array(state.robot_velocity, copy=True)
        t1 = time.perf_counter_ns()
        mpc.append_trajectory_point(point(n_initial + k))
        if k >= args.warmup:
            d = mpc.mpc_debug_data
            tick.append(t1 - t0)
            refs.append(d.duration_horizon_update_ns)
            warm.append(d.duration_generate_warm_start_ns)
            solve.append(d.duration_ocp_solve_ns)
            iters.append(d.ocp.nb_iter)
            qp_iters.append(d.ocp.nb_qp_iter)

    try:
        commit = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            cwd=Path(__file__).parent,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "unknown"
    affinity = os.sched_getaffinity(0)
    policy = {
        os.SCHED_OTHER: "SCHED_OTHER",
        os.SCHED_FIFO: "SCHED_FIFO",
        os.SCHED_RR: "SCHED_RR",
    }.get(os.sched_getscheduler(0), "other")
    load = os.getloadavg()
    print("## Conditions\n")
    print(f"- commit: {commit}")
    print(
        f"- Python {sys.version.split()[0]}, numpy {np.__version__}, pinocchio {pin.__version__}"
    )
    print(f"- CPU: {cpu_model()} ({os.cpu_count()} online cores)")
    print("- n_threads: 1")
    print(
        f"- AC online: {first_line('/sys/class/power_supply/AC/online')}; governor (cpu0): "
        f"{first_line('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')}; "
        f"intel_pstate no_turbo: {first_line('/sys/devices/system/cpu/intel_pstate/no_turbo')}"
    )
    print(
        f"- affinity: {len(affinity)} cpu(s); nice {os.getpriority(os.PRIO_PROCESS, 0)}; scheduler {policy}"
    )
    print(f"- load average at start: {load[0]:.2f} {load[1]:.2f} {load[2]:.2f}")
    print(f"- warm-up ticks {args.warmup}, measured ticks {args.ticks}\n")
    print(
        "| run | n_threads | median us | p90 us | p99 us | max us | references us | "
        "warm start us | solve us | mean iter | mean QP iter |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    print(
        f"| Python MPC.run + integrate | 1 | {median_us(tick):.1f} | "
        f"{percentile_us(tick, 0.9):.1f} | {percentile_us(tick, 0.99):.1f} | "
        f"{percentile_us(tick, 1.0):.1f} | {median_us(refs):.1f} | {median_us(warm):.1f} | "
        f"{median_us(solve):.1f} | {statistics.mean(iters):.2f} | {statistics.mean(qp_iters):.2f} |"
    )


if __name__ == "__main__":
    main()
