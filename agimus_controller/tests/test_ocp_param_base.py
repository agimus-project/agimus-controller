import unittest

from agimus_controller.ocp_param_base import OCPParamsBaseCroco, DTFactorsNSeq


class TestOCPParamsCrocoBase(unittest.TestCase):
    """
    TestOCPParamsCrocoBase unittests parameters settters and getters of OCPParamsBaseCroco class.
    """

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)

    def test_use_filter_line_search_is_a_field(self):
        """use_filter_line_search can be set from the constructor and is stored."""
        kwargs = {
            "dt": 0.01,
            "horizon_size": 10,
            "dt_factor_n_seq": DTFactorsNSeq(factors=[1], n_steps=[10]),
            "solver_iters": 50,
        }
        self.assertFalse(OCPParamsBaseCroco(**kwargs).use_filter_line_search)
        params = OCPParamsBaseCroco(**kwargs, use_filter_line_search=True)
        self.assertTrue(params.use_filter_line_search)

    def test_initialization(self):
        """
        Test the initialization of the OCPParamsBaseCroco class.
        """
        params_dt_factor_n_seq = {"factors": [1], "n_steps": [100]}
        dt_factor_param = DTFactorsNSeq(**params_dt_factor_n_seq)
        params = {
            "dt": 0.01,
            "horizon_size": 100,
            "dt_factor_n_seq": dt_factor_param,
            "solver_iters": 50,
            "qp_iters": 200,
            "termination_tolerance": 1e-3,
            "eps_abs": 1e-6,
            "eps_rel": 0,
            "callbacks": False,
        }
        ocp_param_base_croco = OCPParamsBaseCroco(**params)
        for key, val in params.items():
            res = getattr(ocp_param_base_croco, key)
            self.assertEqual(
                res,
                val,
                f"Value missmatch for parameter '{key}'. Expected: '{val}', got: '{res}'",
            )

        self.assertEqual(params["horizon_size"], ocp_param_base_croco.n_controls)


if __name__ == "__main__":
    unittest.main()
