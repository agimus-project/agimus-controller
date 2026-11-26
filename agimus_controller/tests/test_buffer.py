from copy import deepcopy
import numpy as np
from random import randint
import unittest

from agimus_controller.ocp_param_base import DTFactorsNSeq
from agimus_controller.trajectory import (
    ConstantTrajectoryBuffer,
    TrajectoryBuffer,
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)


class TestTrajectoryBuffer(unittest.TestCase):
    """
    TestOCPParamsCrocoBase unittests parameters settters and getters of OCPParamsBaseCrocoCroco class.
    """

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)
        self.nv = randint(10, 100)  # Number of dof in the robot velocity
        self.nq = self.nv + 1  # Number of dof in the robot configuration

        self.trajectory_size = 100
        self.n_controls = 10
        self.dt_factor_n_seq = DTFactorsNSeq(factors=[1], n_steps=[self.n_controls])
        self.dt = 0.01
        self.dt_ns = int(1e9 * self.dt)

    def generate_random_weighted_states(self, time_ns):
        """
        Generate random data for the TrajectoryPointWeights.
        """
        return WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                time_ns=time_ns,
                robot_configuration=np.random.random(self.nq),
                robot_velocity=np.random.random(self.nv),
                robot_acceleration=np.random.random(self.nv),
                robot_effort=np.random.random(self.nv),
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=np.random.random(self.nv),
                w_robot_velocity=np.random.random(self.nv),
                w_robot_acceleration=np.random.random(self.nv),
                w_robot_effort=np.random.random(self.nv),
            ),
        )

    def test_append_data(self):
        """
        Test adding points to the buffer.
        """
        obj = TrajectoryBuffer(self.dt_factor_n_seq)
        times_ns = np.arange(
            0, 30 * self.trajectory_size * self.dt_ns, self.dt_ns, dtype=int
        )
        for time_ns in times_ns:
            obj.append(self.generate_random_weighted_states(time_ns))

        self.assertEqual(len(obj), times_ns.size)

    def test_clear_past(self):
        """
        Test clearing the past of the buffer.
        """
        obj = TrajectoryBuffer(self.dt_factor_n_seq)
        times_ns = np.arange(
            0, 30 * self.trajectory_size * self.dt_ns, self.dt_ns, dtype=int
        )
        for time_ns in times_ns:
            obj.append(self.generate_random_weighted_states(time_ns))

        obj.clear_past()
        self.assertEqual(len(obj), times_ns.size - 1)
        obj.clear_past()
        self.assertEqual(len(obj), times_ns.size - 2)
        obj.clear_past()
        self.assertEqual(len(obj), times_ns.size - 3)

    def test_compute_horizon_index(self):
        """
        Test computing the time indexes from dt_factor_n_seq.
        """
        dt_factor_n_seq = DTFactorsNSeq(
            factors=[1, 2, 3, 4, 5], n_steps=[2, 2, 2, 2, 2]
        )
        obj = TrajectoryBuffer(dt_factor_n_seq)

        indexes_out = obj.compute_horizon_indexes()
        indexes_test = [0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30]
        np.testing.assert_equal(indexes_out, indexes_test)

    def test_horizon(self):
        """
        Test computing the horizon from the dt_factor_n_seq format.
        """
        obj = TrajectoryBuffer(self.dt_factor_n_seq)
        times_ns = np.arange(
            0, 30 * self.trajectory_size * self.dt_ns, self.dt_ns, dtype=int
        )
        for time_ns in times_ns:
            obj.append(self.generate_random_weighted_states(time_ns))

        horizon = obj.horizon
        self.assertEqual(len(horizon), self.n_controls + 1)
        np.testing.assert_array_equal(
            deepcopy(horizon),
            obj[: self.n_controls + 1],
        )

    def test_horizon_with_more_complex_dt_factor_n_seq(self):
        """
        Test computing the horizon from complex dt_factor_n_seq.
        """
        dt_factor_n_seq = DTFactorsNSeq(
            factors=[1, 2, 3, 4, 5], n_steps=[2, 2, 2, 2, 2]
        )
        horizon_indexes = [0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30]

        obj = TrajectoryBuffer(dt_factor_n_seq)
        self.assertEqual(horizon_indexes, obj.horizon_indexes)

        # Fill the data in
        times_ns = np.arange(
            0, 30 * self.trajectory_size * self.dt_ns, self.dt_ns, dtype=int
        )
        for time_ns in times_ns:
            obj.append(self.generate_random_weighted_states(time_ns))

        # Get the horizon
        horizon = obj.horizon
        self.assertEqual(len(horizon), self.n_controls + 1)
        np.testing.assert_array_equal(
            deepcopy(obj.horizon),
            [obj[index] for index in horizon_indexes],
        )


class TestConstantTrajectoryBuffer(unittest.TestCase):
    """Unit tests for ConstantTrajectoryBuffer."""

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)
        self.nv = randint(10, 100)
        self.nq = self.nv + 1
        self.n_controls = 10
        self.dt_factor_n_seq = DTFactorsNSeq(factors=[1], n_steps=[self.n_controls])
        self.dt = 0.01
        self.dt_ns = int(1e9 * self.dt)

    def _make_point(self, time_ns=0):
        return WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                time_ns=time_ns,
                robot_configuration=np.random.random(self.nq),
                robot_velocity=np.random.random(self.nv),
                robot_acceleration=np.random.random(self.nv),
                robot_effort=np.random.random(self.nv),
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=np.random.random(self.nv),
                w_robot_velocity=np.random.random(self.nv),
                w_robot_acceleration=np.random.random(self.nv),
                w_robot_effort=np.random.random(self.nv),
            ),
        )

    def test_empty_buffer_raises_on_horizon(self):
        """Accessing .horizon before any append must raise AssertionError."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        with self.assertRaises(AssertionError):
            _ = obj.horizon

    def test_empty_buffer_len_is_zero(self):
        """len() of an empty buffer must be 0."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        self.assertEqual(len(obj), 0)

    def test_len_sufficient_after_one_append(self):
        """After a single append, len() must satisfy buffer_has_enough_data for any ratio <= 3."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        obj.append(self._make_point())
        # len must be > horizon_indexes[-1]
        self.assertGreater(len(obj), obj.horizon_indexes[-1])

    def test_horizon_length(self):
        """horizon must return exactly n_controls + 1 elements."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        obj.append(self._make_point())
        self.assertEqual(len(obj.horizon), self.n_controls + 1)

    def test_horizon_all_same_object(self):
        """All elements of horizon must be the same object (no copies)."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        point = self._make_point()
        obj.append(point)
        horizon = obj.horizon
        for h in horizon:
            self.assertIs(h, point)

    def test_append_replaces_latest_point(self):
        """Each append replaces the stored point; horizon reflects the most recent one."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        for i in range(5):
            pt = self._make_point(time_ns=i * self.dt_ns)
            obj.append(pt)
        last_pt = self._make_point(time_ns=99 * self.dt_ns)
        obj.append(last_pt)
        for h in obj.horizon:
            self.assertIs(h, last_pt)

    def test_clear_past_is_noop(self):
        """clear_past must not change len() or the stored point."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        pt = self._make_point()
        obj.append(pt)
        length_before = len(obj)
        obj.clear_past()
        self.assertEqual(len(obj), length_before)
        self.assertIs(obj.horizon[0], pt)

    def test_getitem_always_returns_latest(self):
        """__getitem__ with any index must return the latest point."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        pt = self._make_point()
        obj.append(pt)
        for idx in [0, 5, 100, -1]:
            self.assertIs(obj[idx], pt)

    def test_setitem_updates_latest_point(self):
        """__setitem__ must update the stored point regardless of the index used."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        obj.append(self._make_point())
        new_pt = self._make_point(time_ns=42)
        obj[0] = new_pt
        for h in obj.horizon:
            self.assertIs(h, new_pt)

    def test_pop_returns_latest_point(self):
        """pop() must return the latest point (no removal occurs)."""
        obj = ConstantTrajectoryBuffer(self.dt_factor_n_seq)
        pt = self._make_point()
        obj.append(pt)
        result = obj.pop()
        self.assertIs(result, pt)
        # Point must still be accessible after pop
        self.assertIs(obj.horizon[0], pt)

    def test_complex_dt_factor_n_seq_horizon_length(self):
        """horizon length must equal sum(n_steps) + 1 for non-trivial dt_factor_n_seq."""
        dt_factor_n_seq = DTFactorsNSeq(factors=[1, 2, 3], n_steps=[3, 3, 3])
        obj = ConstantTrajectoryBuffer(dt_factor_n_seq)
        pt = self._make_point()
        obj.append(pt)
        expected_len = sum(dt_factor_n_seq.n_steps) + 1
        self.assertEqual(len(obj.horizon), expected_len)


if __name__ == "__main__":
    unittest.main()
