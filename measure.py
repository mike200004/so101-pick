# Prints the numbers that are hardcoded at the top of ik.c.
# Puts the arm at the zero pose and measures each link in the side view
# (r = distance out from the base, z = height).

import math
import os

import mujoco

SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)


def pivot(name):
    return data.xanchor[model.joint(name).id]


pan = pivot("shoulder_pan")
points = [pivot("shoulder_lift"), pivot("elbow_flex"), pivot("wrist_flex"), data.site("tip").xpos]

print("PAN_X %.5f" % pan[0])
print("SH_R  %.5f" % (points[0][0] - pan[0]))
print("SH_Z  %.5f" % points[0][2])
for i in range(3):
    dr = points[i + 1][0] - points[i][0]
    dz = points[i + 1][2] - points[i][2]
    print("L%d %.5f   A%d %.5f" % (i + 1, math.hypot(dr, dz), i + 1, math.atan2(dz, dr)))
