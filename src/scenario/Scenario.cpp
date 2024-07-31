#include "Scenario.h"

scenario::Scenario::Scenario(const std::string &config_file_path, const std::string &root_path)
{
    try
    {
        YAML::Node node { YAML::LoadFile(root_path + config_file_path) };
        YAML::Node robot_node { node["robot"] };
        std::string type { robot_node["type"].as<std::string>() };
        size_t num_DOFs { robot_node["num_DOFs"].as<size_t>() };
		std::shared_ptr<robots::AbstractRobot> robot;

        if (type == "xarm6")
            robot = std::make_shared<robots::xArm6>(root_path + robot_node["urdf"].as<std::string>(),
                                                    robot_node["gripper_length"].as<float>(),
                                                    robot_node["ground_included"].as<size_t>());
        else if (type == "planar_2DOF")
            robot = std::make_shared<robots::Planar2DOF>(root_path + robot_node["urdf"].as<std::string>());
        else if (type == "planar_10DOF")
            robot = std::make_shared<robots::Planar10DOF>(root_path + robot_node["urdf"].as<std::string>());
        else
            throw std::logic_error("Robot type is not correct!");

        YAML::Node capsules_radius_node { robot_node["capsules_radius"] };
        if (capsules_radius_node.IsDefined())
        {
            if (capsules_radius_node.size() != num_DOFs)
                throw std::logic_error("Number of capsules is not correct!");
                
            std::vector<float> capsules_radius {};
            for (size_t i = 0; i < num_DOFs; i++)
                capsules_radius.emplace_back(capsules_radius_node[i].as<float>());

            robot->setCapsulesRadius(capsules_radius);
        }

        YAML::Node max_vel_node { robot_node["max_vel"] };
        if (max_vel_node.IsDefined())
        {
            if (max_vel_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_vel' is not correct!");

            Eigen::VectorXf max_vel(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_vel(i) = max_vel_node[i].as<float>();

            robot->setMaxVel(max_vel);
        }

        YAML::Node max_acc_node { robot_node["max_acc"] };
        if (max_acc_node.IsDefined())
        {
            if (max_acc_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_acc' is not correct!");

            Eigen::VectorXf max_acc(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_acc(i) = max_acc_node[i].as<float>();

            robot->setMaxAcc(max_acc);
        }

        YAML::Node max_jerk_node { robot_node["max_jerk"] };
        if (max_jerk_node.IsDefined())
        {
            if (max_jerk_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_jerk' is not correct!");

            Eigen::VectorXf max_jerk(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_jerk(i) = max_jerk_node[i].as<float>();
                
            robot->setMaxJerk(max_jerk);
        }

        YAML::Node self_collision_checking_node { robot_node["self_collision_checking"] };
        if (self_collision_checking_node.IsDefined())
            robot->setSelfCollisionChecking(self_collision_checking_node.as<bool>());

        std::shared_ptr<env::Environment> env {std::make_shared<env::Environment>(config_file_path, root_path) };
        std::string state_space { robot_node["space"].as<std::string>() };
        if (state_space == "RealVectorSpace")
            ss = std::make_shared<base::RealVectorSpace>(num_DOFs, robot, env);
        else if (state_space == "RealVectorSpaceFCL")
            ss = std::make_shared<base::RealVectorSpaceFCL>(num_DOFs, robot, env);
        else
            throw std::logic_error("State space does not exist!");

        YAML::Node q_start_node { robot_node["q_start"] };
        YAML::Node q_goal_node { robot_node["q_goal"] };
        if (q_start_node.size() != num_DOFs || q_goal_node.size() != num_DOFs)
            throw std::logic_error("Start or goal size is not correct!");
        
        Eigen::VectorXf q_start_vec(num_DOFs);
        Eigen::VectorXf q_goal_vec(num_DOFs);
        for (size_t i = 0; i < num_DOFs; i++)
        {
            q_start_vec(i) = q_start_node[i].as<float>();
            q_goal_vec(i) = q_goal_node[i].as<float>();
        }
        q_start = std::make_shared<base::RealVectorSpaceState>(q_start_vec);
        q_goal = std::make_shared<base::RealVectorSpaceState>(q_goal_vec);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
}

scenario::Scenario::Scenario(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> q_start_, std::shared_ptr<base::State> q_goal_)
{
    ss = ss_;
    q_start = q_start_;
    q_goal = q_goal_;
    state_space_type = ss->getStateSpaceType();
    num_dimensions = ss->num_dimensions;
}