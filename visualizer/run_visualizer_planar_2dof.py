from log_parser import LogParser
from planar_2dof_vis import visualize
import yaml
from math import pi
import os


if __name__ == "__main__":
    data_file_path = "/RGBTConnect_data/test1.log"

    scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml"
    # scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml"
    # scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml"
    # scenario_file_path = "/data/planar_2dof/scenario3/scenario3.yaml"

    project_path = os.path.dirname(os.path.abspath(__file__))
    project_path = project_path.rsplit('/', 1)[0]
    print(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)

    parser = LogParser(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)
    path = parser.get_path()
    # path = [-2, -2.5]
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    
    with open(project_path + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, 
              image_file=project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path + "planar_2dof.gif", 
              is_trajectory=True, is_dynamic=False, duration=100) 
    