import time
import warnings
import numpy.typing as npt

from agimus_controller.mpc_data import OCPResults, MPCDebugData
from agimus_controller.ocp_base import OCPBase
from agimus_controller.trajectory import (
    TrajectoryBuffer,
    TrajectoryPoint,
    WeightedTrajectoryPoint,
)
from agimus_controller.warm_start_base import WarmStartBase


class MPC(object):
    def __init__(self) -> None:
        self._ocp = None
        self._warm_start = None
        self._mpc_debug_data: MPCDebugData = None
        self._buffer = None
        self._dt_ns = None
        # Playback clock: maps wall time to a position along the reference
        # trajectory. Anchored on the first run() call.
        self._playback_t0_wall_ns = None
        self._playback_t0_traj_ns = None
        self._underrun_logged = False

    def setup(
        self,
        ocp: OCPBase,
        warm_start: WarmStartBase,
        buffer: TrajectoryBuffer,
    ) -> None:
        self._ocp = ocp
        self._warm_start = warm_start
        self._buffer = buffer
        self._mpc_debug_data = MPCDebugData(ocp=self._ocp.debug_data)
        self._dt_ns = int(round(self._ocp.dt * 1e9))

    def reset_playback(self) -> None:
        """Re-anchor the playback clock on the next run() call. Call this when a
        new reference trajectory replaces the current one."""
        self._playback_t0_wall_ns = None
        self._playback_t0_traj_ns = None
        self._underrun_logged = False

    def run(self, initial_state: TrajectoryPoint, current_time_ns: int) -> OCPResults:
        assert self._ocp is not None
        assert self._warm_start is not None
        timer1 = time.perf_counter_ns()

        # Advance the reference by *wall time*, not by one buffer slot per call:
        # extract the horizon at the current playback position so a publisher
        # that fills the buffer slower/faster than the solver rate no longer
        # dilates the trajectory. Falls back to the plain index buffer when no
        # wall clock is supplied (current_time_ns <= 0).
        if current_time_ns and current_time_ns > 0:
            # Only need a single point: horizon_at holds it for the whole
            # horizon on underrun (e.g. once the trajectory has been fully
            # published and the robot is dwelling at the last waypoint).
            if len(self._buffer) < 1:
                return None
            if self._playback_t0_wall_ns is None:
                self._playback_t0_wall_ns = current_time_ns
                self._playback_t0_traj_ns = self._buffer[0].point.time_ns
            t_traj_ns = self._playback_t0_traj_ns + (
                current_time_ns - self._playback_t0_wall_ns
            )
            reference_trajectory, underrun = self._buffer.horizon_at(
                t_traj_ns, self._dt_ns
            )
            if underrun and not self._underrun_logged:
                warnings.warn(
                    "MPC reference underrun: the playback clock ran past the "
                    "last buffered reference point — the trajectory publisher "
                    "is not keeping up with the solver. Holding the last "
                    "reference point.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._underrun_logged = True
        else:
            t_traj_ns = None
            if len(self._buffer) < self._ocp.n_controls + 1:
                return None
            reference_trajectory = self._extract_horizon_from_buffer()

        self._ocp.set_reference_weighted_trajectory(reference_trajectory)
        timer2 = time.perf_counter_ns()

        # TODO avoid building this list by making warm start classes use a reference trajectory with weights.
        reference_trajectory_points = [el.point for el in reference_trajectory]
        x0, x_init, u_init = self._warm_start.generate(
            initial_state, reference_trajectory_points
        )
        assert len(x_init) == self._ocp.n_controls + 1
        assert len(u_init) == self._ocp.n_controls

        timer3 = time.perf_counter_ns()
        self._ocp.solve(x0, x_init, u_init)
        self._warm_start.update_previous_solution(self._ocp.ocp_results)
        if t_traj_ns is None:
            self._buffer.clear_past()
        else:
            self._buffer.clear_before(t_traj_ns)
        timer4 = time.perf_counter_ns()

        # Extract the solution.
        self._mpc_debug_data.ocp = self._ocp.debug_data
        self._mpc_debug_data.reference_id = reference_trajectory_points[0].id
        self._mpc_debug_data.duration_iteration_ns = timer4 - timer1
        self._mpc_debug_data.duration_horizon_update_ns = timer2 - timer1
        self._mpc_debug_data.duration_generate_warm_start_ns = timer3 - timer2
        self._mpc_debug_data.duration_ocp_solve_ns = timer4 - timer3

        return self._ocp.ocp_results

    def integrate(
        self, state: TrajectoryPoint, control: npt.NDArray
    ) -> TrajectoryPoint:
        """Integrate the control starting from state during duration dt.

        Returns:
            the same TrajectoryPoint object, where robot_configuration and robot_velocity have been modified.
        """
        x = self._ocp.integrate(state.robot_state, control)
        state.time_ns += int(self._ocp.dt * 1e9)  # dt is in seconds
        state.robot_configuration = x[: len(state.robot_configuration)]
        state.robot_velocity = x[len(state.robot_configuration) :]
        return state

    @property
    def mpc_debug_data(self) -> MPCDebugData:
        return self._mpc_debug_data

    def append_trajectory_point(self, trajectory_point: WeightedTrajectoryPoint):
        self._buffer.append(trajectory_point)

    def append_trajectory_points(
        self, trajectory_points: list[WeightedTrajectoryPoint]
    ):
        self._buffer.extend(trajectory_points)

    def _extract_horizon_from_buffer(self):
        return self._buffer.horizon
