import types
import unittest
from unittest import mock

import numpy as np
import pinocchio
import pytest

force_feedback_mpc = pytest.importorskip("force_feedback_mpc")

from agimus_controller.ocp.ocp_croco_generic_force_feedback import (  # noqa: E402
  DAMSoftContactAugmentedFwdDynamics,  
  IAMSoftContactAugmented,
)


class FakeIAM:
    """Stands in for the C++ IAM: bounds are infinite and the force constraint off."""

    def __init__(self, differential, step_time, with_cost_residual):
        self.g_lb = np.full(3, -np.inf)
        self.g_ub = np.full(3, np.inf)
        self.with_force_constraint = False
        self.force_lb = self.g_lb.copy()
        self.force_ub = self.g_ub.copy()


class TestIAMSoftContactAugmentedForceBounds(unittest.TestCase):
    def _build(self, **bounds):
        differential = types.SimpleNamespace(
            enabled_directions=(True, True, True),
            build=lambda data: object(),
        )
        model = IAMSoftContactAugmented(differential=differential, **bounds)
        with mock.patch.object(
            force_feedback_mpc, "IAMSoftContactAugmented", FakeIAM, create=True
        ):
            return model.build(None)

    def test_bounds_are_applied_without_dam_constraints(self):
        iam = self._build(force_lb=[-1.0, -2.0, 0.0], force_ub=[1.0, 2.0, 30.0])
        self.assertTrue(iam.with_force_constraint)
        np.testing.assert_array_equal(iam.force_lb, [-1.0, -2.0, 0.0])
        np.testing.assert_array_equal(iam.force_ub, [1.0, 2.0, 30.0])

    def test_no_bounds_leaves_force_constraint_off(self):
        iam = self._build()
        self.assertFalse(iam.with_force_constraint)


class TestDAMSoftContactUpdate(unittest.TestCase):
    def _update(self, **kwargs):
        model = DAMSoftContactAugmentedFwdDynamics(
            costs=[],
            frame_id="tool",
            Kp=[1.0] * 3,
            Kv=[1.0] * 3,
            oPc=(0, 0, 0),
            **kwargs,
        )
        force = pinocchio.Force(np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0]))
        pt = types.SimpleNamespace(
            point=types.SimpleNamespace(forces={"tool": force}),
            weights=types.SimpleNamespace(w_forces={"tool": np.array([4.0, 5.0, 6.0])}),
        )
        dam = types.SimpleNamespace(nc=model._dimension, with_gravity_torque_reg=False)
        model.update(None, dam, pt)
        return dam

    def test_default_enabled_directions_keeps_vector_shape(self):
        dam = self._update()
        np.testing.assert_array_equal(dam.f_des, [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(dam.f_weight, [4.0, 5.0, 6.0])

    def test_single_enabled_direction_selects_axis(self):
        dam = self._update(enabled_directions=(False, False, True))
        np.testing.assert_array_equal(dam.f_des, [3.0])
        np.testing.assert_array_equal(dam.f_weight, [6.0])
