# 1. Introduction
This repository contains Rapid Prototyping Motion Planning Library v2 (RPMPLv2) implemented in C++.

Available planners are:
- Rapidly-exploring Random Trees (RRT-Connect)
- Rapidly-exploring Bur Trees (RBT-Connect)
- Rapidly-exploring Generalized Bur Trees (RGBT-Connect)
- Dynamic Rapidly-exploring Generalized Bur Trees (DRGBT)
- Rapidly-exploring Generalized Bur Multi-Tree (RGBMT*)

The development and test environment are tested on Ubuntu 22.04 + ROS2 Humble.

# 2. How to use
## 2.1 Obtain source code of "RPMPLv2" repository
Choose the location for a target workspace, e.g.:
```
cd ~/
```
DO NOT omit "--recursive"，or the source code of dependent submodules will not be downloaded:
```
git clone https://github.com/roboticsETF/RPMPLv2.git --recursive
```

## 2.2 Update "RPMPLv2" repository
```
cd ~/RPMPLv2
git pull
git submodule sync
git submodule update --init --remote
```

## 2.3 Install required packages
For planning:
```
sudo apt install libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev libyaml-cpp-dev liburdf-dev
```
For visualization:
```
pip3 install trimesh urdfpy
```

## 2.4 Install dependencies
```
cd ~/RPMPLv2
rosdep update
make dependencies
```

## 2.5 Build "RPMPLv2" repository
```
make build
```

# 3. Run the simulation
## 3.1 Set configuration parameters
On the location ```/data/configurations``` you can find the configuration yaml files, which can be modified in order to set the desired configuration parameters for each planner.

For example, open ```configuration_rgbmtstar.yaml```, and set the maximal planning time to 10 sec:
```
MAX_PLANNING_TIME: 10000
```

## 3.2 Set scenario parameters for static planning
Open the folder ```/data```, where you can find three subfolders: ```/planar_2dof```, ```/planar_10dof``` and ```/xarm6```. Inside each of them, you can find different scenario folders containing the corresponding scenario yaml files.

For example, open the file ```/data/planar_2dof/scenario_test/scenario_test.yaml```. You can set as many obstacles as you want. For now, only box obstacles are supported, and you can set:
- ```dim```: Dimensions (x, y and z in [m]);
- ```trans```: Translation (x, y and z in [m]);
- ```rot```: Rotation (quaternion). 

Additionally, by setting ```num_random_obstacles```, you can set as many random obstacles as you want. All of them will be collision free with your start and goal configuration. Note that if you set zero random obstacles, they will not be initialized. 

Moreover, some details about the used robot can be set, such as:
- ```type```: Robot type;
- ```urdf```: Urdf file location;
- ```space```: Configuration space type (RealVectorSpace or RealVectorSpaceFCL);
- ```num_DOFs```: Number of DOFs (number of dimensions of the configuration space);
- ```start```: Start configuration;
- ```goal```: Goal configuration;
- ```gripper_length```: Gripper length in [m] (just set 0 if the gripper is not attached);
- ```capsules_radius```: Radius of each capsule in [m] that approximates robot links;
- ```table_included```: Information whether to include the table on which the robot is mounted. Please check whether 'table' (required as the first obstacle) is (un)commented within 'obstacles'.

Note that the last three options are only available for ```/xarm6``` robot.

For example, if you do not want to use FCL (Flexible Collision Library) in planning, just set:
```
space: "RealVectorSpace"
```
Be aware that, in such case, robot links are approximated with capsules, thus collision checks and distance queries between primitives, capsule and box, are performed. In most cases, this executes faster than when using FCL, especially for more complicated robot mesh structures (containing too many triangles), such as xarm6 structure.

Otherwise, if you do want to use FCL, just set:
```
space: "RealVectorSpaceFCL"
```

## 3.3 Set scenario parameters for dynamic real-time planning
In the following example, we are using DRGBT algorithm. Please, open the file ```/data/xarm6/scenario_real_time/scenario_real_time.yaml```. You can set the following:
- ```num_random_obstacles_init```: Number of random obstacles to start with the testing;
- ```max_num_random_obstacles```: Maximal number of random obstacles to be added;
- ```max_obs_vel```: Maximal velocity of each obstacle in [m/s];
- ```num_test_init```: Number of testing to start with (default: 1);
- ```num_success_test_init```: Number of successful tests so far (default: 0);
- ```max_num_tests```: Maximal number of tests that should be carried out.

For example, if you set ```num_random_obstacles_init: 1``` and ```max_num_random_obstacles: 100```, the number of obstacles will be: 1, 2, 3, ..., 10, 20, 30, ..., 100. On the other hand, you can optionally add predefined obstacles within ```obstacles```, as described in Subsection 3.2. That will specify only their initial position. Note that in case you do not want to use random obstacles, just set ```num_random_obstacles_init : 0``` and ```max_num_random_obstacles : 0```.

You can define the way how obstacles move within the file ```Environment.cpp``` in the function ```updateEnvironment```. Currently, each obstacle follows a straight random line until it reaches the workspace limit, when it returns back by randomly changing direction. During the motion, each obstacle can randomly change its velocity between zero and ```max_obs_vel```.

Parameters ```num_test_init``` and ```num_success_test_init``` are useful if you suddenly abort the testing. Afterwards, you can continue where you left just by setting these two parameters.

In the file ```/data/configurations/configuration_drgbt```, you can set the following DRGBT parameters:
- ```MAX_NUM_ITER```: Maximal number of algorithm iterations;
- ```MAX_ITER_TIME```: Maximal runtime of a single iteration in [ms]. Be aware that the obstacle covers a distance of ```max_obs_vel * MAX_ITER_TIME``` in [mm] during a single iteration;
- ```MAX_PLANNING_TIME```: Maximal algorithm runtime in [ms];
- ```INIT_HORIZON_SIZE```: Initial horizon size. Default: 10.
- ```MAX_ANG_VEL```: Maximal angular velocity of each robot's joint in [rad/s], which determines an advancing step in C-space in [rad] when moving from current towards next state in a single iteration. Default: 3.1415 for xArm6;
- ```TRESHOLD_WEIGHT```: Treshold for the replanning assessment. Range: between 0 and 1. Default: 0.5;
- ```D_CRIT```: Critical distance in W-space to compute critical nodes;
- ```MAX_NUM_MODIFY_ATTEMPTS```: Maximal number of attempts when modifying bad or critical states. Default: 10;
- ```STATIC_PLANNER_NAME```: Name of a static planner (for obtaining the predefined path). Default: "RGBTConnect" or "RGBMT*";
- ```REAL_TIME_SCHEDULING```: Available real-time schedulings are: "FPS" - Fixed Priority Scheduling; "DPS" - Dynamic Priority Scheduling; If you set "", no real-time scheduling will be used;
- ```MAX_TIME_TASK1```: Maximal time in [ms] which Task 1 (computing the next configuration) can take from the processor. It must be less than ```MAX_ITER_TIME```. Default: 20.
 
Finally, in the file ```/apps/test_drgbt.cpp```, you can set via ```routines``` which routines' execution times should be stored during the testing. File ```/data/xarm6/scenario_real_time/scenario_real_time_routine_times<number>.log``` will contain all logged execution times.

## 3.4 Test planners
All test files are available within the folder ```/apps```. For example, open ```test_rgbmtstar.cpp```. You can set the file path of desired scenario via ```scenario_file_path```, and maximal number of tests in ```max_num_tests```. 

In the new tab type:
```
cd ~/RPMPLv2/install/rpmpl_library/apps
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

## 3.5 Visualize the robot and obstacles
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
