"""Jacobian cross-validation: MuJoCo vs central differences."""

import mujoco
import numpy as np
import pytest
from helpers import central_diff_jac, random_valid_qpos, set_urdf_qpos


@pytest.mark.parametrize("site", ["left_fr3v2_1_hand_tcp", "right_fr3v2_1_hand_tcp"])
def test_jacobian_matches_finite_difference(base_model, urdf, site, seed=77):
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(base_model)
    jacp = np.zeros((3, base_model.nv))
    jacr = np.zeros((3, base_model.nv))
    max_err = 0.0
    for trial in range(10):
        values = random_valid_qpos(base_model, rng)
        set_urdf_qpos(base_model, data, urdf, values)
        mujoco.mj_forward(base_model, data)
        qpos = data.qpos.copy()
        site_id = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_SITE, site)
        mujoco.mj_jacSite(base_model, data, jacp, jacr, site_id)
        fd = central_diff_jac(base_model, site, qpos)
        err = np.abs(jacp - fd).max()
        max_err = max(max_err, err)
        assert err <= 1e-5, f"{site} trial {trial}: jac err {err}"
    print(f"{site}: max jacobian error {max_err:.2e}")
