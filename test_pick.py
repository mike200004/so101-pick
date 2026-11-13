# run with: make && pytest

import random

import mujoco

import pick
from ik import ik


def test_ik_puts_the_tip_where_asked():
    model = mujoco.MjModel.from_xml_path(pick.SCENE)
    data = mujoco.MjData(model)
    for x, y, z in [(0.2, 0.0, 0.05), (0.25, 0.1, 0.02), (0.15, -0.12, 0.03), (0.12, 0.2, 0.06)]:
        q = ik(x, y, z)
        assert q is not None
        for name, value in zip(pick.ARM, q):
            data.joint(name).qpos = value
        mujoco.mj_forward(model, data)
        tip = data.site("tip").xpos
        assert abs(tip[0] - x) < 0.001
        assert abs(tip[1] - y) < 0.001
        assert abs(tip[2] - z) < 0.001


def test_ik_says_no_when_too_far():
    assert ik(0.6, 0.0, 0.05) is None
    assert ik(0.2, 0.0, 0.3) is None


def test_picks_up_random_cubes():
    random.seed(0)
    model = mujoco.MjModel.from_xml_path(pick.SCENE)
    data = mujoco.MjData(model)
    for i in range(5):
        x, y, yaw = pick.random_cube()
        pick.setup(model, data, x, y, yaw)
        plan = pick.make_plan(x, y, yaw)
        while data.time < pick.plan_length(plan) + 0.5:
            data.ctrl[:] = pick.target_at(plan, data.time)
            mujoco.mj_step(model, data)
        assert pick.in_tray(data)
