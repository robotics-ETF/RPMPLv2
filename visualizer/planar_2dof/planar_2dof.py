import math
from urdfpy import URDF
import pyrender
from state_spaces.real_vector_space import RealVectorSpace
from trimesh.creation import box, cylinder
from trimesh.collision import CollisionManager
import trimesh.transformations as transforms
import numpy as np
import threading
import time


class Planar2DOF(RealVectorSpace):
    def __init__(self, obstacles) -> None:
        self.robot = URDF.load('../data/planar_2dof/planar_2dof.urdf')
        self.spaces = RealVectorSpace(2)
        self.robot_cm = CollisionManager()
        self.env_cm = CollisionManager()
        self.start_config = [0, 0]
        self.camera_pos_z = 3.0

        self.traj = []
        self.count_i = 0
        self.curr_q = [0, 0]

        cfg = self.get_config([0, 0])
        fk = self.robot.link_fk(cfg=cfg)
        self.init_poses = []
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose, link_mesh = self.get_link_mesh(tm)
            self.init_poses.append(init_pose)
            self.robot_cm.add_object(
                tm.name, link_mesh, np.matmul(pose, init_pose))

        for i, ob in enumerate(obstacles):
            self.env_cm.add_object("obstacle_" + str(i), ob)
            self.env_cm.add_object("prediction_" + str(i), ob)

    def set_trajectory(self, path):
        self.traj = path
        self.curr_q = path[0]
        self.count_i = 0

    def get_next_q(self):
        self.count_i += 1
        if self.count_i >= len(self.traj):
            return None

        self.curr_q = self.traj[self.count_i]
        return self.curr_q

    def update_env(self, obstacles):
        with threading.Lock():
            for i, ob in enumerate(obstacles):
                self.env_cm.remove_object("obstacle_" + str(i))
                self.env_cm.add_object("obstacle_" + str(i), ob)

    def update_predictions(self, predictions):
        with threading.Lock():
            for i, ob in enumerate(predictions):
                self.env_cm.remove_object("prediction_" + str(i))
                self.env_cm.add_object("prediction_" + str(i), ob)

    def spaces(self):
        return self.spaces

    def robot(self):
        return self.robot

    def is_in_collision(self, q):
        cfg = self.get_config(q)
        fk = self.robot.link_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose = self.init_poses[i]
            self.robot_cm.set_transform(tm.name, np.matmul(pose, init_pose))

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        return self.robot_cm.in_collision_other(self.env_cm)

    def distance_to_obstacles(self, q):
        cfg = self.get_config(q)
        fk = self.robot.link_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose = self.init_poses[i]
            self.robot_cm.set_transform(tm.name, np.matmul(pose, init_pose))
            print(tm.name)
            print(np.matmul(pose, init_pose))

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        min_dist, names, data = self.robot_cm.min_distance_other(self.env_cm, return_names=True, return_data=True)
        return (min_dist, names, data.point(names[0]), data.point(names[1]))

    def distance(self, q):
        cfg = self.get_config(q)
        fk = self.robot.link_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            pose = fk[tm]
            init_pose = self.init_poses[i]
            self.robot_cm.set_transform(tm.name, np.matmul(pose, init_pose))

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        return self.robot_cm.min_distance_other(self.env_cm)

    def is_valid(self, q, qe=None, num_checks=None):
        if qe is None or num_checks is None:
            res = self.is_in_collision(q)
            return not(res)
        else:
            for i in range(num_checks):
                q_i = self.get_qnew(q, qe, i/num_checks)
                res = self.is_in_collision(q_i)
                if res:
                    return False
        return True

    def get_config(self, q):
        if len(self.robot.actuated_joints) != len(q):
            raise Exception("Wrong num. of dimensions of q")

        cfg = {}
        for i in range(len(q)):
            cfg[self.robot.actuated_joints[i].name] = q[i]

        return cfg

    def get_link_mesh(self, tm):
        init_pose = tm.visuals[0].origin
        if tm.visuals[0].geometry.cylinder is not None:
            length = tm.visuals[0].geometry.cylinder.length
            radius = tm.visuals[0].geometry.cylinder.radius
            mesh = cylinder(radius, length)
            mesh.visual.face_color = tm.visuals[0].material.color
            return init_pose, mesh
        else:
            ext = tm.visuals[0].geometry.box.size
            mesh = box(ext)
            mesh.visual.face_colors = tm.visuals[0].material.color
            return init_pose, mesh

    def show(self, q=None, obstacles=None, image_file=None):
        cfg = self.get_config(q)

        # adding robot to the scene
        scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02, 1.0])
        fk = self.robot.link_fk(cfg=cfg)
        self.visualize_configuration(scene, fk)

        # adding base box to the scene
        # ground = box([0.5, 0.5, 0.01])
        # ground.apply_translation([0, 0, -0.1])

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        # nc = pyrender.Node(camera=cam, matrix=np.eye(4))
        # scene.add_node(nc)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = self.camera_pos_z
        scene.add(cam, pose=init_cam_pose)

        light = pyrender.DirectionalLight(color=np.ones(3), intensity=1.0)
        scene.add(light, pose=init_cam_pose)

        # scene.add(pyrender.Mesh.from_trimesh(ground))

        # adding obstacles to the scene
        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        if (image_file is not None):
            from matplotlib import pyplot as plt
            r = pyrender.OffscreenRenderer(640, 480, point_size=1.0)
            color, _ = r.render(scene)
            plt.figure(figsize=(20,20))
            plt.axis('off')
            plt.imshow(color)
            plt.savefig(image_file)
        else:
            pyrender.Viewer(scene, viewport_size = (1400, 1050), use_raymond_lighting=True)

    def animate(self, q_traj=None, obstacles=None, fps=10.0, image_file=None):
        cfgs = self.interpolate(q_traj)

        # Create the scene
        scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02, 1.0])
        fk_start = self.robot.link_fk(cfg=cfgs[0])
        fk_goal = self.robot.link_fk(cfg=cfgs[len(cfgs)-1])
        node_map, init_pose_map = self.visualize_configuration(scene, fk_start)
        self.visualize_configuration(scene, fk_start, [255,0,255,100])
        self.visualize_configuration(scene, fk_goal, [0,255,0,100])

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = self.camera_pos_z
        scene.add(cam, pose=init_cam_pose)

        light = pyrender.DirectionalLight(color=np.ones(3), intensity=1.0)
        scene.add(light, pose=init_cam_pose)

        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        # Pop the visualizer asynchronously
        v = pyrender.Viewer(scene, viewport_size = (1400, 1050), run_in_thread=True, use_raymond_lighting=True, record=True)
        time.sleep(1.0)
        
        # Now, run our loop
        saved = False
        i = 0
        while v.is_active:            
            if i < len(cfgs) - 1:
                i += 1
            else:
                i = 0
                if not saved:
                    saved = True
                    if image_file is not None:
                        v.close_external()
                        v.save_gif(image_file)
                time.sleep(1.0)
            cfg = cfgs[i]
            fk = self.robot.link_fk(cfg=cfg)
            v.render_lock.acquire()
            for j, mesh in enumerate(fk):
                if j < 3:
                    pose = fk[mesh]
                    node_map[mesh].matrix = np.matmul(pose, init_pose_map[mesh])
            v.render_lock.release()
            time.sleep(1.0 / fps)
    
    def animate_dynamic(self, q_traj=None, obstacles=None, fps=10.0, image_file=None):
        cfgs = [self.get_config(q) for q in q_traj]

        # Create the scene
        scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02, 1.0])
        fk_start = self.robot.link_fk(cfg=cfgs[0])
        fk_goal = self.robot.link_fk(cfg=cfgs[len(cfgs)-1])
        node_map, init_pose_map = self.visualize_configuration(scene, fk_start)
        self.visualize_configuration(scene, fk_start, [255,0,255,100])
        self.visualize_configuration(scene, fk_goal, [0,255,0,100])

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = self.camera_pos_z
        scene.add(cam, pose=init_cam_pose)

        light = pyrender.DirectionalLight(color=np.ones(3), intensity=1.0)
        scene.add(light, pose=init_cam_pose)

        M = []
        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False), name="Obstacles")
            M.append(transforms.identity_matrix())
        
        # Pop the visualizer asynchronously
        v = pyrender.Viewer(scene, viewport_size = (1400, 1050), run_in_thread=True, use_raymond_lighting=True, record=True)
        time.sleep(1.0)
        
        # Now, run our loop
        saved = False
        i = 0
        OB = v.scene.get_nodes(name="Obstacles")
        while v.is_active:
            v.render_lock.acquire()
            M = self.update_obstacles(M)
            for j, ob in enumerate(OB):
                v.scene.set_pose(ob, M[j])
                
            if i < len(cfgs) - 1:
                i += 1
            else:
                i = 0
                if not saved:
                    saved = True
                    if image_file is not None:
                        v.close_external()
                        v.save_gif(image_file)
                time.sleep(1.0)
            cfg = cfgs[i]
            fk = self.robot.link_fk(cfg=cfg)
            for j, mesh in enumerate(fk):
                if j < 3:
                    pose = fk[mesh]
                    node_map[mesh].matrix = np.matmul(pose, init_pose_map[mesh])
            v.render_lock.release()
            time.sleep(1.0 / fps)

    def interpolate(self, q_traj, step=0.1):
        cfgs = []
        idx = 0
        while idx < len(q_traj) - 1:
            q1 = np.array(q_traj[idx])
            q2 = np.array(q_traj[idx + 1])
            D = np.linalg.norm(q2 - q1)
            cfgs.append(self.get_config(q1))
            for i in range(0, math.floor(D / step)):
                q1 += step * (q2 - q1) / np.linalg.norm(q2 - q1)
                cfgs.append(self.get_config(q1))
            idx += 1
        cfgs.append(self.get_config(q2))
        return cfgs

    def update_obstacles(self, M):
        for i in range(len(M)):
            M[i][0,3] -= 0.01
        return M
    
    def visualize_configuration(self, scene, fk, color=None):
        node_map = {}
        init_pose_map = {}
        for i, tm in enumerate(fk):
            if i == 3:
                break
            init_pose, link_mesh = self.get_link_mesh(tm)
            if not color == None:
                link_mesh.visual.vertex_colors = color
            mesh = pyrender.Mesh.from_trimesh(link_mesh, smooth=False)
            node_map[tm] = scene.add(mesh, pose=np.matmul(fk[tm], init_pose))
            init_pose_map[tm] = init_pose
        return node_map, init_pose_map