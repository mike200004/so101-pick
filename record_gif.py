# Saves demo.gif of one pick (needs pillow)

import random

import mujoco
from PIL import Image

import pick

random.seed(3)
model = mujoco.MjModel.from_xml_path(pick.SCENE)
data = mujoco.MjData(model)
x, y, yaw = pick.random_cube()
pick.setup(model, data, x, y, yaw)
plan = pick.make_plan(x, y, yaw)

renderer = mujoco.Renderer(model, 240, 320)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0.13, 0.06, 0.09]
cam.distance, cam.azimuth, cam.elevation = 0.75, 215, -25

fps = 12
frames = []
while data.time < pick.plan_length(plan):
    data.ctrl[:] = pick.target_at(plan, data.time)
    mujoco.mj_step(model, data)
    if len(frames) < data.time * fps:
        renderer.update_scene(data, cam)
        frames.append(Image.fromarray(renderer.render()))

frames[0].save("demo.gif", save_all=True, append_images=frames[1:], duration=1000 // fps, loop=0, optimize=True)
print("saved demo.gif,", len(frames), "frames")
