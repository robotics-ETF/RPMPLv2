from xml.etree.ElementTree import PI
from log_parser import LogParser
from spatial_10dof_vis import visualize
import yaml
from math import pi
import os

if __name__ == "__main__":
    data_file_path = "/DRGBT_data/test4_1.log"
    # data_file_path = "/RRTx_data/test4_1.log"
    # data_file_path = "/MARS_data/MARS_trajectory_scenario10dof_v1.log"

    scenario_file_path = "/data/spatial_10dof/scenario_random_obstacles/scenario_test.yaml"

    project_path = os.path.dirname(os.path.abspath(__file__))
    project_path = project_path.rsplit('/', 1)[0]
    print(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)

    parser = LogParser(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)
    path = parser.get_path()
    # path = [1,-1,1,-1,1,-1,1,-1,1,-1]
    #for p in path:
    #    print(p)
    #print(path)
    # visualize(path, "test.png")
    
    with open(project_path + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, 
              image_file=project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path + "spatial_10dof.gif", 
              is_trajectory=True, is_dynamic=True, duration=50) 
    