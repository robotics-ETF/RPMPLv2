#import os
#os.environ["PYOPENGL_PLATFORM"] = "osmesa"

from trimesh.creation import box
import trimesh.transformations as transforms
from planar_2dof.planar_2dof import Planar2DOF
from math import pi
import time

def visualize(q=None, obstacles=None, image_file=None, is_trajectory=False, is_dynamic=False, duration=100):
    #obstacles = [box([0.2, 1.2, 0.1])]
    #obstacles[0].apply_translation([1.3+0.25, 0, 0])
    obstacles = obstacles['environment']
    for i, obs in enumerate(obstacles):
        obs = obs['box']
        dim = obs['dim']
        pos = obs['pos']
        rot = [0,0,0,1]
        obstacles[i] = box(dim)
        M = transforms.quaternion_matrix(rot)
        M[0:3,3] = pos
        # print(M)
        obstacles[i].apply_transform(M)

    planar_2dof = Planar2DOF(obstacles)
    robot = planar_2dof.robot
    for link in robot.actuated_joints:
        print(link.name)
    print("Number of joints: ", len(robot.actuated_joints))

    if is_dynamic:
        planar_2dof.animate_dynamic(q, obstacles=obstacles, duration=duration, image_file=image_file)
    elif not is_trajectory:
        planar_2dof.show(q, obstacles, image_file)
    else:
        planar_2dof.animate(q, obstacles=obstacles, duration=duration, image_file=image_file)
    