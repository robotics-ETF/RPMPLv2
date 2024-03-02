#import os
#os.environ["PYOPENGL_PLATFORM"] = "osmesa"

from trimesh.creation import box
from xarm6.xarm6 import Xarm6
from math import pi
import time

def visualize(q=None, obstacles=None, image_file=None, is_trajectory=False, is_dynamic=False, fps=10.0):
    #obstacles = [box([0.2, 1.2, 0.1])]
    #obstacles[0].apply_translation([1.3+0.25, 0, 0])
    obstacles = obstacles['environment']
    for i, obs in enumerate(obstacles):
        obs = obs['box']
        dim = obs['dim']
        trans = obs['pos']
        obstacles[i] = box(dim)
        obstacles[i].apply_translation(trans)

    xarm6 = Xarm6(obstacles)
    robot = xarm6.robot
    for link in robot.actuated_joints:
        print(link.name)
    print("Number of joints: ", len(robot.actuated_joints))

    if is_dynamic:
        xarm6.animate_dynamic(q, obstacles=obstacles, fps=fps, image_file=image_file)
    elif not is_trajectory:
        xarm6.show(q, obstacles, image_file)
    else:
        xarm6.animate(q, obstacles=obstacles, fps=fps, image_file=image_file)
