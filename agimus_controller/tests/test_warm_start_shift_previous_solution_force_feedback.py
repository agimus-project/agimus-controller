import types
import unittest

import numpy as np
import pinocchio

from agimus_controller.trajectory import TrajectoryPoint
from agimus_controller.warm_start_shift_previous_solution_force_feedback import (
    WarmStartShiftPreviousSolutionForceFeedback,
)


class TestWarmStartForceFeedbackGenerate(unittest.TestCase):
    def _generate(self, enabled_directions):
        ws = WarmStartShiftPreviousSolutionForceFeedback()
        ws.shift = lambda: None  # only the initial force handling is tested
        ws._enabled_directions = enabled_directions
        ws._frame_id = "tool"
        ws._previous_solution = types.SimpleNamespace(
            states=[np.zeros(1)], feed_forward_terms=[np.zeros(1)]
        )
        initial_state = TrajectoryPoint(
            robot_configuration=np.array([0.1, 0.2]),
            robot_velocity=np.array([0.3, 0.4]),
            forces={"tool": pinocchio.Force(np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0]))},
        )
        x0, _, _ = ws.generate(initial_state, [])
        return x0

    def test_3d_force_is_read_from_the_frame_entry(self):
        x0 = self._generate((True, True, True))
        np.testing.assert_array_equal(x0, [0.1, 0.2, 0.3, 0.4, 1.0, 2.0, 3.0])

    def test_1d_force_selects_the_enabled_axis(self):
        x0 = self._generate((False, False, True))
        np.testing.assert_array_equal(x0, [0.1, 0.2, 0.3, 0.4, 3.0])
