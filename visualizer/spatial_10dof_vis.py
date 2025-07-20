#import os
#os.environ["PYOPENGL_PLATFORM"] = "osmesa"

from trimesh.creation import box
import trimesh.transformations as transforms
from spatial_10dof.spatial_10dof import Spatial10DOF
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
        M = transforms.quaternion_matrix(rot)
        M[0:3,3] = pos
        # print(M)
        obstacles[i].apply_transform(M)
        positions.append(np.array(pos))
        velocities.append(np.array(vel))

    spatial_10dof = Spatial10DOF(obstacles, positions, velocities)
    robot = spatial_10dof.robot
    for link in robot.actuated_joints:
        print(link.name)
    print("Number of joints: ", len(robot.actuated_joints))

    if is_dynamic:
        spatial_10dof.animate_dynamic(q, obstacles=obstacles, duration=duration, image_file=image_file)
    elif not is_trajectory:
        spatial_10dof.show(q, obstacles, image_file)
    else:
        spatial_10dof.animate(q, obstacles=obstacles, duration=duration, image_file=image_file)
    