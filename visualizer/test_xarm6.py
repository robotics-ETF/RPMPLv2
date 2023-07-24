from trimesh.creation import box
from xarm6.xarm6 import Xarm6
from math import pi
import time


obstacles = [box([0.2, 1.2, 0.1])]
obstacles[0].apply_translation([1, 0, 0])

xarm6 = Xarm6(obstacles)
robot = xarm6.robot


for link in robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.actuated_joints))

fk = robot.link_fk()
print(fk[robot.links[0]])

print(type(obstacles[0]))
xarm6.show(obstacles=obstacles, q=[-pi/2, 0, 0, 0, 0, 0], use_collision=False)

ret = xarm6.is_valid(q=[-pi/2, 0, 0, 0, 0, 0])
print(ret)
