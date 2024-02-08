//
// Created by dinko on 17.02.22.
//

#include "Scenario.h"
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpaceState.h"
#include "Planar2DOF.h"
#include "Planar10DOF.h"
#include "xArm6.h"
#include "ConfigurationReader.h"

scenario::Scenario::Scenario(std::string configuration_file, std::string root_path)
{
    YAML::Node node = YAML::LoadFile(root_path + configuration_file);
    std::vector<env::Obstacle> obstacles(node["obstacles"].size());
    for (int i = 0; i < node["obstacles"].size(); i++)
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
        robot = std::make_shared<robots::xArm6>(root_path + robot_node["urdf"].as<std::string>(),
                                                robot_node["gripper_length"].as<float>(),
                                                robot_node["table_included"].as<bool>());
    else if (type == "planar_2DOF")
        robot = std::make_shared<robots::Planar2DOF>(root_path + robot_node["urdf"].as<std::string>());
    else if (type == "planar_10DOF")
        robot = std::make_shared<robots::Planar10DOF>(root_path + robot_node["urdf"].as<std::string>());

    YAML::Node capsules_radius_node = robot_node["capsules_radius"];
    if (capsules_radius_node.IsDefined())
    {
        if (capsules_radius_node.size() != num_DOFs)
            throw std::logic_error("Number of capsules is not correct!");
            
        std::vector<float> capsules_radius;
        for (int i = 0; i < capsules_radius_node.size(); i++)
            capsules_radius.emplace_back(capsules_radius_node[i].as<float>());

        robot->setCapsulesRadius(capsules_radius);
    }

    YAML::Node max_vel_node = robot_node["max_vel"];
    if (max_vel_node.IsDefined())
    {
        if (max_vel_node.size() != num_DOFs)
            throw std::logic_error("The size of 'max_vel' is not correct!");

        std::vector<float> max_vel;
        for (int i = 0; i < max_vel_node.size(); i++)
            max_vel.emplace_back(max_vel_node[i].as<float>());

        robot->setMaxVel(max_vel);
    }

    YAML::Node max_acc_node = robot_node["max_acc"];
    if (max_acc_node.IsDefined())
    {
        if (max_acc_node.size() != num_DOFs)
            throw std::logic_error("The size of 'max_acc' is not correct!");

        std::vector<float> max_acc;
        for (int i = 0; i < max_acc_node.size(); i++)
            max_acc.emplace_back(max_acc_node[i].as<float>());

        robot->setMaxAcc(max_acc);
    }

    YAML::Node max_jerk_node = robot_node["max_jerk"];
    if (max_jerk_node.IsDefined())
    {
        if (max_jerk_node.size() != num_DOFs)
            throw std::logic_error("The size of 'max_jerk' is not correct!");

        std::vector<float> max_jerk;
        for (int i = 0; i < max_jerk_node.size(); i++)
            max_jerk.emplace_back(max_jerk_node[i].as<float>());
            
        robot->setMaxJerk(max_jerk);
    }

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
    
    for (int i = 0; i < start_node.size(); i++)
    {
        start_vec(i) = start_node[i].as<float>();
        goal_vec(i) = goal_node[i].as<float>();
    }
    start = std::make_shared<base::RealVectorSpaceState>(start_vec);
    goal = std::make_shared<base::RealVectorSpaceState>(goal_vec);

}
