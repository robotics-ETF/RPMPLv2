# 1. Introduction
This repository contains Rapid Prototyping Motion Planning Library v2 (RPMPLv2) implemented in C++.

Available planners are:
- Rapidly-exploring Random Trees (RRT-Connect)
- Rapidly-exploring Bur Trees (RBT-Connect)
- Rapidly-exploring Generalized Bur Trees (RGBT-Connect)
- Rapidly-exploring Generalized Bur Multi-Tree star (RGBMT*)
- Dynamic Rapidly-exploring Generalized Bur Trees (DRGBT)

The development and test environment are tested on Ubuntu 22.04 + ROS2 Humble.

# 2. How to use
## 2.1 Use docker and devcontainer

If VS Code (or any other IDE with a support for VS Code's devcontainers), then just open and use "Rebuild and open in container" option. Then proceed with 2.6. Otherwise, continue with 2.2.

## 2.2 Obtain source code of "RPMPLv2" repository
Choose the location for a target workspace, e.g.:
```
cd ~/
```
DO NOT omit "--recursive"，or the source code of dependent submodules will not be downloaded:
```
git clone https://github.com/roboticsETF/RPMPLv2.git --recursive
```

## 2.3 Update "RPMPLv2" repository
```
cd ~/RPMPLv2
git pull
git submodule sync
git submodule update --init --remote
```

## 2.4 Install required packages
For planning:
```
sudo apt install libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev libyaml-cpp-dev liburdf-dev
```
For visualization:
```
pip3 install trimesh urdfpy
```

## 2.5 Install dependencies
```
cd ~/RPMPLv2
rosdep update
make dependencies
```

## 2.6 Build "RPMPLv2" repository
```
make build
```

# 3. Run the simulation
## 3.1 Set configuration parameters
On the location ```/data/configurations``` you can find the configuration yaml files, which can be modified in order to set the desired configuration parameters for each planner.

For example, open ```configuration_rgbmtstar.yaml```, and set the maximal planning time to 10 seconds:
```
MAX_PLANNING_TIME: 10
```

## 3.2 Set scenario parameters for static planning
Open the folder ```/data```, where you can find three subfolders: ```/planar_2dof```, ```/planar_10dof``` and ```/xarm6```. Inside each of them, you can find different scenario folders containing the corresponding scenario yaml files.

For example, open the file ```/data/planar_2dof/scenario_test/scenario_test.yaml```. You can set as many objects as you want in the ```environment``` node. You can set the following:
- ```label```: Label to name the object (e.g., ```ground```, ```dynamic_obstacle```, ```picking_object```, etc.). If not specified, it will be an empty string;
- ```dim```: Dimensions of the object (x, y and z in [m]);
- ```pos```: Position of the object (x, y and z in [m]);
- ```rot```: Rotation of the object (x, y, z in [m], and w in [rad]), specified as quaternion. If not specified, the object will be AABB (axis-aligned bounding-box);

Additionally, by setting ```num``` in ```random_obstacles``` node, you can set as many random obstacles as you want with dimensions ```dim```. All of them will be collision free with your start (and goal) configuration. Note that if you set zero random obstacles, they will not be initialized. 

Moreover, some details about the used robot can be set in the ```robot``` node, such as:
- ```type```: Robot type;
- ```urdf```: Urdf file location;
- ```space```: Configuration space type (RealVectorSpace or RealVectorSpaceFCL);
- ```num_DOFs```: Number of DOFs (number of dimensions of the configuration space);
- ```q_start```: Start configuration;
- ```q_goal```: Goal configuration;
- ```capsules_radius```: Radius of each capsule in [m] that approximates a corresponding robot's link;
- ```gripper_length```: Gripper length in [m] (just set 0 if the gripper is not attached);
- ```ground_included```: Information whether to include ground on which the robot is mounted. Please check whether 'ground' is added in ```environment```. When set to '0', ground is not included. Otherwise, when set to a number greater than zero, it determines the starting robot's link that may collide with ground;
- ```self_collision_checking```: Whether self-collision should be checked;
- ```WS_center```: Workspace center point in [m];
- ```WS_radius```: Workspace radius in [m] assuming spherical workspace shape;
- ```max_vel```: Maximal velocity of each robot's joint in [rad/s] for revolute joints, or in [mm/s] for prismatic joints;
- ```max_acc```: Maximal acceleration of each robot's joint in [rad/s²] for revolute joints, or in [mm/s²] for prismatic joints;
- ```max_jerk```: Maximal jerk of each robot's joint in [rad/s³] for revolute joints, or in [mm/s³] for prismatic joints.

Note that ```gripper_length``` and ```ground_included``` are only available for ```/xarm6``` robot. Moreover, parameters ```max_vel```, ```max_acc``` and ```max_jerk``` can be ommited, if not relevant in the planning. 

Parameters ```WS_center``` and ```WS_radius``` are relevant only when random obstacles exist. Otherwise, they can be ommited.

Total number of tests/runs can be specified by ```max_num``` within the ```testing``` node.

For example, if you do not want to use FCL (Flexible Collision Library) in planning, just set:
```
space: "RealVectorSpace"
```
Be aware that, in such case, robot links are approximated with capsules, thus collision checks and distance queries between primitives, capsule and AABB, are performed. In most cases, this executes faster than when using FCL, especially for more complicated robot mesh structures (containing too many triangles), such as xarm6 structure.

Otherwise, if you do want to use FCL, just set:
```
space: "RealVectorSpaceFCL"
```

## 3.3 Set scenario parameters for dynamic real-time planning
In the following example, we are using DRGBT algorithm. Please, open the file ```/data/xarm6/scenario_real_time/scenario_real_time.yaml```. You can set the following in the ```random_obstacles``` node:
- ```init_num```: Number of random obstacles to start with the testing;
- ```max_num```: Maximal number of random obstacles to be added;
- ```max_vel```: Maximal velocity of each obstacle in [m/s];
- ```max_acc```: Maximal acceleration of each obstacle in [m/s²];
- ```dim```: Dimensions of each random obstacle in [m].

For example, if you set ```init_num: 1``` and ```max_num: 100```, the number of obstacles will be: 1, 2, 3, ..., 10, 20, 30, ..., 100. On the other hand, you can optionally add predefined obstacles within ```obstacles```, as described in Subsection 3.2. That will specify only their initial position. Note that in case you do not want to use random obstacles, just set ```init_num: 0``` and ```max_num: 0```.

You can define the way how obstacles move within the file ```Environment.cpp``` in the function ```updateEnvironment```. Currently, each obstacle follows a straight random line (when ```max_acc``` is set to 0) until it reaches the workspace limit, when it returns back by randomly changing direction. During the motion, each obstacle can randomly change its velocity between zero and ```max_vel```.

Additionally, if you want a start ```q_start``` and a goal ```q_goal``` configuration to be generated randomly, you can define a minimal workspace distance ```min_dist_start_goal``` between them within ```robot``` node. If you set ```min_dist_start_goal: 0```, the start and goal configuration will be fixed, thus they must be specified within ```q_start``` and ```q_goal```. The workspace distance between two configurations is computed as a sum of distances between their skeleton points over a middle skeleton, which is determined by an averaged configuration between the start and the goal. 

Moreover, you can set the following in the ```testing``` node:
- ```init_num```: Number of testing to start with (default: 1);
- ```init_num_success```: Initial number of already achieved successful tests (default: 0);
- ```max_num```: Maximal number of tests that should be carried out;
- ```reach_successful_tests```: If true, run totally ```max_num``` successful tests.

Parameters ```init_num``` and ```init_num_success``` are useful if you suddenly abort the testing. Afterwards, you can continue where you left just by setting these two parameters.

In the file ```/data/configurations/configuration_drgbt.yaml```, you can set the following DRGBT parameters:
- ```MAX_NUM_ITER```: Maximal number of algorithm iterations;
- ```MAX_ITER_TIME```: Maximal runtime of a single iteration in [s]. Be aware that the obstacle covers a distance of ```max_vel * MAX_ITER_TIME``` (when ```max_acc``` is set to 0) in [m] during a single iteration;
- ```MAX_PLANNING_TIME```: Maximal algorithm runtime in [s];
- ```INIT_HORIZON_SIZE```: Initial horizon size. Default: 10.
- ```TRESHOLD_WEIGHT```: Treshold for the replanning assessment. Range: between 0 and 1. Default: 0.5;
- ```D_CRIT```: Critical distance in W-space to compute critical nodes;
- ```MAX_NUM_MODIFY_ATTEMPTS```: Maximal number of attempts when modifying bad or critical states. Default: 10;
- ```STATIC_PLANNER_TYPE```: Type of a static planner (for obtaining the predefined path). Available planners: "RGBMT*", "RGBT-Connect", "RBT-Connect" and "RRT-Connect";
- ```REAL_TIME_SCHEDULING```: Available real-time scheduling is "FPS" - Fixed Priority Scheduling; If you set "None", no real-time scheduling will be used;
- ```MAX_TIME_TASK1```: Maximal time in [s] which Task 1 (computing the next configuration) can take from the processor. It must be less than ```MAX_ITER_TIME```. Default: 0.020;
- ```TRAJECTORY_INTERPOLATION```: Method for interpolation of trajectory: 'None' or 'Spline'. If 'None' is used, the robot always moves at its highest speed, i.e., an advancing step for moving from 'q_current' towards 'q_next' in C-space is determined by maximal robot's velocity. On the other hand, if 'Spline' is used, then a quintic spline from 'q_current' to 'q_next' is computed in order to satisfy all constaints on robot's maximal velocity, acceleration and jerk. All configuration parameters considering splines can be set in the file ```/data/configurations/configuration_splines.yaml```.
- ```GUARANTEED_SAFE_MOTION```: Whether robot motion is surely safe for environment. If collision eventually occurs, it will be at robot's zero velocity, meaning that an obstacle hit the robot, and not vice versa. This feature is intended to be used only for real/practical applications, thus it can be used only when ```TRAJECTORY_INTERPOLATION``` is set to 'Spline'.


Finally, in the file ```/apps/test_drgbt.cpp```, you can set via ```routines``` which routines' execution times should be stored during the testing. File ```/data/xarm6/scenario_real_time/scenario_real_time_routine_times<number>.log``` will contain all logged execution times.

## 3.4 Test planners
All test files are available within the folder ```/apps```. For example, open ```test_rgbmtstar.cpp```. You can set the file path of desired scenario via ```scenario_file_path```, and maximal number of tests in ```max_num_tests```. 

In the new tab type:
```
cd ~/RPMPLv2/build/rpmpl_library/apps
```

Test RRT-Connect:
```
./test_rrtconnect
```

Test RBT-Connect:
```
./test_rbtconnect
```

Test RGBT-Connect:
```
./test_rgbtconnect
```

Test RGBMT*:
```
./test_rgbmtstar
```

Test DRGBT:
```
./test_drgbt
```

After the planning is finished, all log files (containing all details about the planning) will be stored in ```/data``` folder (e.g., ```/data/planar_2dof/scenario_test/scenario_test_planner_data.log```).

## 3.5 Visualize the robot and environment
In the new tab type:
```
cd ~/xarm6-etf-lab/build
```

Visualize plannar_2dof robot:
```
python3 visualizer/run_visualizer_planar_2dof.py
```
![scenario2_planar_2dof](https://github.com/roboticsETF/RPMPLv2/assets/126081373/89c7e908-ca23-4bdc-9b89-9803051749a5)
![scenario_test_planar_2dof_nice_example](https://github.com/roboticsETF/RPMPLv2/assets/126081373/cfc0aba7-4b0c-4b4f-bfaf-d10afc184f4b)

Visualize plannar_10dof robot:
```
python3 visualizer/run_visualizer_planar_10dof.py
```
![scenario1_planar_10dof](https://github.com/roboticsETF/RPMPLv2/assets/126081373/209d7ec7-f91d-4ce7-a339-df1705e71fc4)
![scenario2_planar_10dof](https://github.com/roboticsETF/RPMPLv2/assets/126081373/24e6bde6-841a-4a64-b3b0-7254f41d7e07)

Visualize xarm6 robot:
```
python3 visualizer/run_visualizer_xarm6.py
```
![scenario1_xarm6_v1](https://github.com/roboticsETF/RPMPLv2/assets/126081373/cc3bd662-8d6e-4ecc-a992-8646e2808490)
![scenario2_xarm6_v2](https://github.com/roboticsETF/RPMPLv2/assets/126081373/40e71a54-a7b3-4546-a4c6-f052a7c426d4)
![scenario_test_xarm6_nice_example_v2](https://github.com/roboticsETF/RPMPLv2/assets/126081373/2289cdba-4b2b-49b7-a517-79ed6387d988)

The visualization gif files will be stored in ```/data``` folder (e.g., ```/data/planar_2dof/scenario_test/scenario_test_planar_2dof.gif```).
