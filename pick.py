# SO-101 picks up a cube and drops it in the tray.
#
#   python pick.py          # watch it in the MuJoCo viewer
#   python pick.py 50       # run 50 random cubes without a window, print how many worked

import math
import os
import random
import sys

import mujoco

from ik import ik

SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

OPEN = 0.7
CLOSED = 0.0     # a bit past where the jaws touch the cube, so they squeeze it
JAW_OFFSET = 0.008  # see make_plan
HOME = [0.2, 0.0, 0.06]
TRAY = [0.12, 0.2]
PAN_X = 0.03884  # same as in ik.c


def joint_targets(pos, roll, grip):
    q = ik(pos[0], pos[1], pos[2])
    if q is None:
        print("can't reach", pos)
        sys.exit(1)
    return q + [roll, grip]


def grasp_roll(x, y, yaw):
    # turn the wrist so the jaws line up with the cube's sides.
    # the jaws close along the line from the base to the gripper, so the
    # wrist has to make up the difference between that and the cube's yaw.
    a = yaw - math.atan2(y, x - PAN_X)
    while a > math.pi / 4:
        a -= math.pi / 2
    while a < -math.pi / 4:
        a += math.pi / 2
    return a


def make_plan(x, y, yaw):
    # each step: (seconds, tip position, wrist roll, gripper)
    roll = grasp_roll(x, y, yaw)

    # the SO-101 gripper is lopsided: the fixed jaw sits behind the wrist
    # axis and only the other jaw moves. so aim a bit behind the cube,
    # that way the fixed jaw comes down just behind it instead of on top.
    a = math.atan2(y, x - PAN_X) + roll
    gx = x - JAW_OFFSET * math.cos(a)
    gy = y - JAW_OFFSET * math.sin(a)

    above = [gx, gy, 0.06]
    down = [gx, gy, 0.016]
    over_tray = [TRAY[0], TRAY[1], 0.06]
    plan = [
        (1.5, above, roll, OPEN),
        (1.0, down, roll, OPEN),
        (0.6, down, roll, CLOSED),
        (1.0, above, roll, CLOSED),
        (1.5, over_tray, 0.0, CLOSED),
        (0.6, over_tray, 0.0, OPEN),
        (1.5, HOME, 0.0, OPEN),
    ]
    return plan


def target_at(plan, t):
    # walk along the plan, moving the tip in a straight line between steps.
    # cosine easing so it starts and stops smoothly.
    prev_pos, prev_roll, prev_grip = HOME, 0.0, OPEN
    for secs, pos, roll, grip in plan:
        if t < secs:
            s = 0.5 - 0.5 * math.cos(math.pi * t / secs)
            p = [a + s * (b - a) for a, b in zip(prev_pos, pos)]
            r = prev_roll + s * (roll - prev_roll)
            g = prev_grip + s * (grip - prev_grip)
            return joint_targets(p, r, g)
        t -= secs
        prev_pos, prev_roll, prev_grip = pos, roll, grip
    return joint_targets(prev_pos, prev_roll, prev_grip)


def plan_length(plan):
    return sum(step[0] for step in plan)


def random_cube():
    r = random.uniform(0.14, 0.24)
    a = random.uniform(-0.8, 0.3)
    x = PAN_X + r * math.cos(a)
    y = r * math.sin(a)
    yaw = random.uniform(0, math.pi / 2)
    return x, y, yaw


def setup(model, data, x, y, yaw):
    # put the cube and the arm where they start
    mujoco.mj_resetData(model, data)
    data.joint("cube").qpos = [x, y, 0.0125, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    start = joint_targets(HOME, 0.0, OPEN)
    for name, q in zip(ARM, start):
        data.joint(name).qpos = q
    data.ctrl[:] = start
    mujoco.mj_forward(model, data)


def in_tray(data):
    x, y, z = data.body("cube").xpos
    return abs(x - TRAY[0]) < 0.045 and abs(y - TRAY[1]) < 0.045 and z < 0.03


def run_headless(n):
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    ok = 0
    for i in range(n):
        x, y, yaw = random_cube()
        setup(model, data, x, y, yaw)
        plan = make_plan(x, y, yaw)
        while data.time < plan_length(plan) + 0.5:
            data.ctrl[:] = target_at(plan, data.time)
            mujoco.mj_step(model, data)
        if in_tray(data):
            ok += 1
        else:
            print("missed: cube at %.3f %.3f yaw %.2f" % (x, y, yaw))
    print("%d / %d cubes in the tray" % (ok, n))


def run_viewer():
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    x, y, yaw = random_cube()
    setup(model, data, x, y, yaw)
    plan = make_plan(x, y, yaw)

    def control(m, d):
        d.ctrl[:] = target_at(plan, d.time)

    # control callback runs inside the viewer loop, so plain `python` works on macOS
    mujoco.set_mjcb_control(control)
    try:
        mujoco.viewer.launch(model, data)
    finally:
        mujoco.set_mjcb_control(None)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run_headless(int(sys.argv[1]))
    else:
        run_viewer()
