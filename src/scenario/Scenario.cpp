//
// Created by dinko on 17.02.22.
//

#include "Scenario.h"
#include "Environment.h"
#include "AbstractRobot.h"
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpaceState.h"
#include "xArm6.h"
#include "Planar2DOF.h"
#include "Planar10DOF.h"

#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"
#include <stdexcept>

#include <glog/logging.h>

scenario::Scenario::Scenario(std::string configuration_file, std::string root_path)
{
    YAML::Node node = YAML::LoadFile(root_path + configuration_file);
    std::vector<env::Obstacle> obstacles(node["obstacles"].size());
    for (size_t i = 0; i < node["obstacles"].size(); ++i)
	{
        YAML::Node obstacle = node["obstacles"][i];
        if(obstacle["box"].IsDefined())
        {
            // BOX
            YAML::Node box_size = obstacle["box"]["dim"];
            float bx = box_size[0].as<float>();
            float by = box_size[1].as<float>();
            float bz = box_size[2].as<float>();
            fcl::Boxf obs(bx, by, bz);
            
            YAML::Node trans = obstacle["box"]["trans"];
            float tx = trans[0].as<float>();
            float ty = trans[1].as<float>();
            float tz = trans[2].as<float>();
            // LOG(INFO) << "read trans";

            YAML::Node rot = obstacle["box"]["rot"];
            float rx = rot[1].as<float>();
            float ry = rot[2].as<float>();
            float rz = rot[3].as<float>();
            float rw = rot[0].as<float>();
            
            fcl::Vector3f tr(tx, ty, tz);
            fcl::Quaternionf quat(rw, rx, ry, rz);
            fcl::Transform3f tf(fcl::Transform3f::Identity()); 
            tf.linear() = quat.matrix(); 
            tf.translation() = tr;
            // LOG(INFO) << "Object tf: " << tf.matrix();
            
            // obstacles[i] = env::Obstacle(std::make_pair(obs, tf));
            obstacles[i] = std::make_pair(obs, tf);
        }
    }
    env = std::make_shared<env::Environment>(obstacles);

    YAML::Node robot_node = node["robot"];
    std::string type = robot_node["type"].as<std::string>();
    int num_DOFs = robot_node["num_DOFs"].as<int>();
    if (type == "xarm6")
    {
        YAML::Node capsules_radius_node = robot_node["capsules_radius"];
        if (capsules_radius_node.size() != num_DOFs)
            throw std::logic_error("Number of capsules is not correct!");
            
        std::vector<float> capsules_radius;
        for (size_t i = 0; i < capsules_radius_node.size(); ++i)
            capsules_radius.emplace_back(capsules_radius_node[i].as<float>());

        robot = std::make_shared<robots::xArm6>(root_path + robot_node["urdf"].as<std::string>(),
                                                capsules_radius,
                                                robot_node["gripper_length"].as<float>(),
                                                robot_node["table_included"].as<bool>());
    }
    else if (type == "planar_2DOF")
        robot = std::make_shared<robots::Planar2DOF>(root_path + robot_node["urdf"].as<std::string>());
    else if (type == "planar_10DOF")
        robot = std::make_shared<robots::Planar10DOF>(root_path + robot_node["urdf"].as<std::string>());

    std::string space_state = robot_node["space"].as<std::string>();
    if (space_state == "RealVectorSpace")
        ss = std::make_shared<base::RealVectorSpace>(num_DOFs, robot, env);
    else if (space_state == "RealVectorSpaceFCL")
        ss = std::make_shared<base::RealVectorSpaceFCL>(num_DOFs, robot, env);

    YAML::Node start_node = robot_node["start"];
    YAML::Node goal_node = robot_node["goal"];
    Eigen::VectorXf start_vec(num_DOFs);
    Eigen::VectorXf goal_vec(num_DOFs);
    if (start_node.size() != goal_node.size())
        throw std::logic_error("Start and goal size mismatch!");
    for (size_t i = 0; i < start_node.size(); ++i)
    {
        start_vec(i) = start_node[i].as<float>();
        goal_vec(i) = goal_node[i].as<float>();
    }
    start = std::make_shared<base::RealVectorSpaceState>(start_vec);
    goal = std::make_shared<base::RealVectorSpaceState>(goal_vec);

}
