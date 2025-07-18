#import os
#os.environ["PYOPENGL_PLATFORM"] = "osmesa"

from trimesh.creation import box
from xarm6.xarm6 import Xarm6
from math import pi
import time
import numpy as np

def visualize(q=None, obstacles=None, image_file=None, is_trajectory=False, is_dynamic=False, duration=100):
    # Initial positions and velocities
    positions = []
    velocities = []

    #obstacles = [box([0.2, 1.2, 0.1])]
    #obstacles[0].apply_translation([1.3+0.25, 0, 0])
    obstacles = obstacles['environment']
    for i, obs in enumerate(obstacles):
        obs = obs['box']
        dim = obs['dim']
        pos = obs['pos']
        vel = obs['vel']
        rot = [0,0,0,1]
        obstacles[i] = box(dim)
        obstacles[i].apply_translation(pos)
        positions.append(np.array(pos))
        velocities.append(np.array(vel))

    xarm6 = Xarm6(obstacles, positions, velocities)
    robot = xarm6.robot
    for link in robot.actuated_joints:
        print(link.name)
    print("Number of joints: ", len(robot.actuated_joints))

    if is_dynamic:
        xarm6.animate_dynamic(q, obstacles=obstacles, duration=duration, image_file=image_file)
    elif not is_trajectory:
        xarm6.show(q, obstacles, image_file)
    else:
        xarm6.animate(q, obstacles=obstacles, duration=duration, image_file=image_file)
