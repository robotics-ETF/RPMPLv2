#include "Environment.h"

env::Environment::Environment(const std::shared_ptr<env::Environment> env)
{
    objects = env->getObjects();
    WS_center = env->getWSCenter();
    WS_radius = env->getWSRadius();
    base_radius = env->getBaseRadius();
    robot_max_vel = env->getRobotMaxVel();
    ground_included = env->getGroundIncluded();
}

env::Environment::Environment(const std::string &config_file_path, const std::string &root_path)
{
    YAML::Node node { YAML::LoadFile(root_path + config_file_path) };
    size_t num_added { 0 };

    try
    {
        for (size_t i = 0; i < node["environment"].size(); i++)
        {
            std::shared_ptr<env::Object> object;
            YAML::Node obstacle = node["environment"][i];
            if (obstacle["box"].IsDefined())
            {
                std::string label = "";
                if (obstacle["box"]["label"].IsDefined())
                    label = obstacle["box"]["label"].as<std::string>();

                if (label == "ground" && node["robot"]["ground_included"].as<size_t>() == 0)
                    continue;

                YAML::Node d = obstacle["box"]["dim"];
                YAML::Node p = obstacle["box"]["pos"];
                YAML::Node r = obstacle["box"]["rot"];
                YAML::Node min_dist_tol = obstacle["box"]["min_dist_tol"];
                
                fcl::Vector3f dim(d[0].as<float>(), d[1].as<float>(), d[2].as<float>());
                fcl::Vector3f pos(p[0].as<float>(), p[1].as<float>(), p[2].as<float>());
                fcl::Quaternionf rot = fcl::Quaternionf::Identity();

                if (r.IsDefined())
                    rot = fcl::Quaternionf(r[3].as<float>(), r[0].as<float>(), r[1].as<float>(), r[2].as<float>());

                object = std::make_shared<env::Box>(dim, pos, rot, label);

                if (min_dist_tol.IsDefined())
                    object->setMinDistTol(min_dist_tol.as<float>());

            }
            else if (obstacle["sphere"].IsDefined())
            {
                // TODO
            }
            else
                throw std::domain_error("Object type is wrong! ");

            objects.emplace_back(object);
            std::cout << "Added " << num_added++ << ". " << object;
        }

        if (node["robot"]["ground_included"].IsDefined())
            ground_included = node["robot"]["ground_included"].as<size_t>();
        else if (node["robot"]["type"].as<std::string>() == "xarm6")
        {
            ground_included = 0;
            throw std::domain_error("It is not defined whether ground is included! Thus, it will not be included. ");
        }

        if (node["robot"]["WS_center"].IsDefined())
        {
            for (size_t i = 0; i < 3; i++)
                WS_center(i) = node["robot"]["WS_center"][i].as<float>();

            WS_radius = node["robot"]["WS_radius"].as<float>();
        }
        else
            throw std::domain_error("Workspace center point is not defined! ");
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
}

env::Environment::~Environment()
{
    objects.clear();
}

void env::Environment::addObject(const std::shared_ptr<env::Object> object, const fcl::Vector3f &velocity, 
                                 const fcl::Vector3f &acceleration) 
{
    object->setVelocity(velocity);
    object->setAcceleration(acceleration);
    objects.emplace_back(object);
}

// Remove object at 'idx' position
void env::Environment::removeObject(size_t idx)
{
    objects.erase(objects.begin() + idx);
}

// Remove objects from 'start_idx'-th object to 'end_idx'-th object
// If 'end_idx' is not passed, it will be considered as the last index in 'objects'
void env::Environment::removeObjects(int start_idx, int end_idx)
{
    if (end_idx == -1)
        end_idx = objects.size() - 1;
    
    for (int idx = end_idx; idx >= start_idx; idx--)
        objects.erase(objects.begin() + idx);
}

// Remove objects with label 'label' if 'with_label' is true (default)
// Remove objects NOT with label 'label' if 'with_label' is false
void env::Environment::removeObjects(const std::string &label, bool with_label)
{
    if (with_label)
    {
        for (int idx = objects.size()-1; idx >= 0; idx--)
        {
            if (objects[idx]->getLabel() == label)
                objects.erase(objects.begin() + idx);
        }
    }
    else
    {
        for (int idx = objects.size()-1; idx >= 0; idx--)
        {
            if (objects[idx]->getLabel() != label)
                objects.erase(objects.begin() + idx);
        }
    }        
}

// Remove all objects from the environment
void env::Environment::removeAllObjects()
{
    objects.clear();
}

// Check whether an object position 'pos' is valid when the object moves at 'vel' velocity
bool env::Environment::isValid(const Eigen::Vector3f &pos, float vel)
{
    float tol_radius {std::max(vel / robot_max_vel, base_radius)};

    if (ground_included > 0)
    {
        if ((pos - WS_center).norm() > WS_radius || pos.z() < 0 ||              // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z()) ||     // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                              // Surrounding of robot base
            return false;
    }
    else
    {
        if ((pos - WS_center).norm() > WS_radius ||                                                     // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z() && pos.z() > -base_radius) ||   // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                                                      // Surrounding of robot base
            return false;
    }

    return true;
}

// ------------------------------------- Set the law for obstacles motion here ------------------------------------- //

// Straight-line motion
// void env::Environment::updateEnvironment(float delta_time)
// {
//     fcl::Vector3f pos {};
//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         if (objects[i]->getLabel() != "dynamic_obstacle")
//             continue;

//         pos = objects[i]->getPosition();
//         pos.x() -= delta_time * objects[i]->getMaxVel();    // Move along x-axis
//         objects[i]->setPosition(pos);
//         std::cout << i << ". " << objects[i];
//     }
// }

// Circular motion 
// Intented for "/data/xarm6/scenario1/scenario1.yaml"
// void env::Environment::updateEnvironment(float delta_time)
// {
//     fcl::Vector3f pos {};
//     float phi {};
//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         if (objects[i]->getLabel() != "dynamic_obstacle")
//             continue;

//         pos = objects[i]->getPosition();
//         phi = std::atan2(pos.y(), pos.x());
//         objects[i]->setVelocity(Eigen::Vector3f(-std::sin(phi), std::cos(phi), 0.0) * objects[i]->getMaxVel());
//         pos += objects[i]->getVelocity() * delta_time;
//         objects[i]->setPosition(pos);
//         // std::cout << i << ". " << objects[i];
//     }
// }

// Specific motion 
// Intented for "/data/xarm6/scenario1/scenario2.yaml"
// void env::Environment::updateEnvironment(float delta_time)
// {
//     fcl::Vector3f pos {};
//     const float path_len_max { 1 };

//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         if (objects[i]->getLabel() != "dynamic_obstacle")
//             continue;

//         if (path_len > path_len_max)
//         {
//             sign *= -1;
//             path_len = -path_len_max;
//         }
        
//         pos = objects[i]->getPosition();
//         if (pos.y() > 0)
//             objects[i]->setVelocity(Eigen::Vector3f::UnitX() * sign * objects[i]->getMaxVel());     // Move along x-axis
//         else
//             objects[i]->setVelocity(Eigen::Vector3f::UnitY() * sign * objects[i]->getMaxVel());     // Move along y-axis

//         path_len += objects[i]->getVelocity().norm() * delta_time;
//         pos += objects[i]->getVelocity() * delta_time;
//         objects[i]->setPosition(pos);
//         // std::cout << i << ". " << objects[i];
//     }
// }

// Reflect obstacles in random directions 
// Intented for "/data/xarm6/scenario_random_obstacles/scenario_random_obstacles.yaml"
// void env::Environment::updateEnvironment(float delta_time)
// {
//     float vel_intensity {};
//     fcl::Vector3f pos {}, vel {};

//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         // std::cout << objects[i];
//         if (objects[i]->getLabel() != "dynamic_obstacle")
//             continue;

//         vel = objects[i]->getVelocity() + objects[i]->getAcceleration() * delta_time;
//         pos = objects[i]->getPosition() + vel * delta_time;
//         vel_intensity = vel.norm();
//         // std::cout << i << ". position: " << pos.transpose() << "\n";
//         // std::cout << i << ". vel_intensity: " << vel_intensity << "\n";

//         if (vel_intensity > objects[i]->getMaxVel())
//         {
//             // std::cout << i << ". Invalid object velocity. Computing new acceleration.\n";
//             fcl::Vector3f acc = fcl::Vector3f::Random(3);
//             acc.normalize();
//             objects[i]->setAcceleration(objects[i]->getAcceleration().norm() * acc);
//             i--;
//         }
//         else if (!isValid(pos, vel_intensity))
//         {
//             // std::cout << i << ". position: " << pos.transpose() << "\n";
//             // std::cout << i << ". Invalid object position. Computing new velocity.\n";
//             vel = fcl::Vector3f::Random(3);
//             vel.normalize();
//             objects[i]->setVelocity(vel_intensity * vel);
//             i--;
//         }
//         else
//         {
//             objects[i]->setVelocity(vel);
//             objects[i]->setPosition(pos);
//             // std::cout << i << ". position successfully computed: " << pos.transpose() << "\n";
//             // std::cout << i << ". " << objects[i];
//         }
//     }
//     // std::cout << "-------------------------------------------------" << std::endl;
// }

// Intented for "/data/xarm6/scenario_random_obstacles/scenario_random_obstacles.yaml"
// Reflect obstacles according to the principle of light reflecting
void env::Environment::updateEnvironment(float delta_time)
{
    float tol_radius {}, t_param {};
    fcl::Vector3f pos_next {}, vec_normal {};
    bool change { true };

    for (size_t i = 0; i < objects.size(); i++)
    {
        // std::cout << objects[i];
        if (objects[i]->getLabel() != "dynamic_obstacle")
            continue;

        tol_radius = std::max(objects[i]->getVelocity().norm() / robot_max_vel, base_radius);
        pos_next = objects[i]->getPosition() + objects[i]->getVelocity() * delta_time;
        change = true;
        
        if (pos_next.z() < 0)
            vec_normal << 0, 0, 1;
        else if ((pos_next - WS_center).norm() > WS_radius)
            vec_normal << -pos_next.x(), -pos_next.y(), -(pos_next.z() - WS_center.z());
        else if (pos_next.head(2).norm() < tol_radius && pos_next.z() < WS_center.z())
            vec_normal << pos_next.x(), pos_next.y(), 0;
        else if ((pos_next - WS_center).norm() < tol_radius)
            vec_normal << pos_next.x(), pos_next.y(), pos_next.z() - WS_center.z();
        else
        {
            objects[i]->setPosition(pos_next);
            change = false;
        }

        if (change)
        {
            t_param = (pos_next - objects[i]->getPosition()).dot(vec_normal) / vec_normal.squaredNorm();
            objects[i]->setPosition(2*pos_next - objects[i]->getPosition() - 2*t_param * vec_normal);
            objects[i]->setVelocity((objects[i]->getPosition() - pos_next) / delta_time);
        }
        // std::cout << i << ". position: " << objects[i]->getPosition().transpose() << "\n";
        // std::cout << i << ". velocity: " << objects[i]->getVelocity().transpose() << "\n";
        // std::cout << i << ". vel_intensity: " << objects[i]->getVelocity().norm() << "\n";
    }
    // std::cout << "-------------------------------------------------" << std::endl;
}
