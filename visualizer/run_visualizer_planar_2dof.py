from log_parser import LogParser
from planar_2dof_vis import visualize
import yaml
from math import pi


if __name__ == "__main__":
    scenario_file_path = "data/planar_2dof/scenario_test/scenario_test.yaml"
    # scenario_file_path = "data/planar_2dof/scenario1/scenario1.yaml"
    # scenario_file_path = "data/planar_2dof/scenario2/scenario2.yaml"
    # scenario_file_path = "data/planar_2dof/scenario3/scenario3.yaml"

    parser = LogParser("../" + scenario_file_path[0:len(scenario_file_path)-5] + "_planner_data.log")
    path = parser.get_path()
    # path = [-2, -2.5]
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    
    with open("../" + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file="../" + scenario_file_path[0:len(scenario_file_path)-5] + "_planar_2dof.gif", 
              is_trajectory=True, is_dynamic=True, fps=10.0) 
    