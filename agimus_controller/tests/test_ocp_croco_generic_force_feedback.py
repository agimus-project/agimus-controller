import types
import unittest

import numpy as np
import pinocchio
import pytest

pytest.importorskip("force_feedback_mpc")

from agimus_controller.ocp.ocp_croco_generic_force_feedback import (  # noqa: E402
    DAMSoftContactAugmentedFwdDynamics,
)


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
