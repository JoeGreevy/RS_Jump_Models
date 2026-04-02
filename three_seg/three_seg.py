import opensim as osim
import sys
import os
sys.path.insert(0, os.getcwd())
from utils.four_dof import four_dof

# ======= Parameters ======= #
thigh_len, shank_len, foot_len = 0.4, 0.4, 0.2

print("Creating model...")
model = osim.Model()
model.setName('three_seg')
model.setUseVisualizer(True);

# ==== Bodies ==== #
print("Adding bodies...")
ground = model.getGround()
thigh = osim.Body('thigh', 8.0, osim.Vec3(0), osim.Inertia(0))
shank = osim.Body('shank', 3.0, osim.Vec3(0), osim.Inertia(0))
foot = osim.Body('foot', 1.3, osim.Vec3(0), osim.Inertia(0))

# === Joints === #
print("Adding joints...")
st = four_dof()
print(st)
hip = osim.CustomJoint('hip', ground, osim.Vec3(0, 3, 0), osim.Vec3(0), thigh, osim.Vec3(0, 0, 0), osim.Vec3(0), st)

# === Assemble the model === #
model.addBody(thigh)
model.addJoint(hip)

print("Initializing state ...")
state = model.initSystem()



# model.print('three_seg.osim')

