//
// Created by dinko on 14.02.22.
//

#include "Environment.h"

#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

#include <glog/logging.h>

typedef std::shared_ptr<fcl::CollisionGeometry<float>> CollisionGeometryPtr;

env::Environment::Environment(const std::string &filename)
{
    YAML::Node node = YAML::LoadFile(filename);
    std::vector<std::shared_ptr<fcl::CollisionObject<float>>> parts;
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
            CollisionGeometryPtr fclBox(new fcl::Box<float>(bx, by, bz));

            YAML::Node trans = obstacle["box"]["trans"];
            float tx = trans[0].as<float>();
            float ty = trans[1].as<float>();
            float tz = trans[2].as<float>();

            YAML::Node rot = obstacle["box"]["rot"];
            float rx = rot[1].as<float>();
            float ry = rot[2].as<float>();
            float rz = rot[3].as<float>();
            float rw = rot[0].as<float>();
            
            fcl::Vector3f tr(tx, ty, tz);
            fcl::Quaternionf quat(rw, rx, ry, rz);

            //std::cout << "Object tf: " << quat << "\n" << tr << "\n------------";
            std::shared_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject<float>(fclBox, quat.matrix(), tr));
            ob->computeAABB();
            parts.emplace_back(ob);
            velocities.emplace_back(fcl::Vector3f::Zero());
            acc_signs.emplace_back(0);
        }
    }        
}

env::Environment::Environment(const fcl::Box<float> &box, const fcl::Transform3<float> &tf)
{
    CollisionGeometryPtr fclBox(new fcl::Box(box.side[0], box.side[1], box.side[2]));
	std::shared_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject(fclBox, tf));
    ob->computeAABB();
    parts.emplace_back(ob);
    velocities.emplace_back(fcl::Vector3f::Zero());
    acc_signs.emplace_back(0);
    std::cout << "Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
}

env::Environment::Environment(const std::vector<env::Obstacle> obs)
{
    for (size_t i = 0; i < obs.size(); ++i)
    {
        CollisionGeometryPtr fclBox(new fcl::Box<float>(obs[i].first.side[0], obs[i].first.side[1], obs[i].first.side[2]));
        std::shared_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject(fclBox, obs[i].second));
        ob->computeAABB();
        parts.emplace_back(ob);
        velocities.emplace_back(fcl::Vector3f::Zero());
        acc_signs.emplace_back(0);
        std::cout << i << ". Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
    }
}

env::Environment::~Environment()
{
    parts.clear();
    velocities.clear();
    acc_signs.clear();
}

void env::Environment::setParts(const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &parts_)
{
    parts.clear();
    parts = parts_;
    for (std::shared_ptr<fcl::CollisionObjectf> part : parts)
        part->computeAABB();
    
    velocities = std::vector<fcl::Vector3f>(parts.size(), fcl::Vector3f::Zero());
    acc_signs = std::vector<int>(parts.size(), 0);
}

void env::Environment::addCollisionObject(const std::shared_ptr<fcl::CollisionObject<float>> ob, const fcl::Vector3f &velocity,
    float acc_sign) 
{
    ob->computeAABB();
    parts.emplace_back(ob);
    velocities.emplace_back(velocity);
    acc_signs.emplace_back(acc_sign);
}

void env::Environment::removeCollisionObjects(int start_idx)
{
    for (int idx = parts.size()-1; idx >= start_idx; idx--)
    {
        parts.erase(parts.begin() + idx);
        velocities.erase(velocities.begin() + idx);
        acc_signs.erase(acc_signs.begin() + idx);
    }
}

bool env::Environment::isValid(const Eigen::Vector3f &pos, float obs_vel)
{
    float tol_radius = std::max(obs_vel / robot_max_vel, base_radius);
    if ((pos - WS_center).norm() > WS_radius || pos.z() < 0 ||          // Out of workspace
        pos.head(2).norm() < tol_radius && pos.z() < WS_center.z() ||   // Surrounding of robot base
        (pos - WS_center).norm() < tol_radius)                          // Surrounding of robot base
        return false;

    return true;
}

// void env::Environment::updateEnvironment(float delta_time)
// {
//     fcl::Vector3f pos;
//     for (int i = 0; i < parts.size(); i++)
//     {
//         pos = parts[i]->getTranslation();
//         pos(0) -= delta_time * max_vel;    // Move along x axis
//         parts[i]->setTranslation(pos);
//         parts[i]->computeAABB();
//         // std::cout << i << ". Obstacle range: (" << parts[i]->getAABB().min_.transpose() << ")\t(" << parts[i]->getAABB().max_.transpose() << ")\n";
//     }
// }

void env::Environment::updateEnvironment(float delta_time)
{
    float vel_new;
    fcl::Vector3f pos, velocity;

    // TODO: Podesiti da se samo pomijeraju prepreke one koje imaju labelu npr. "moving_object"
    for (int i = 1; i < parts.size(); i++)  // Table will be the first object
    {
        vel_new = velocities[i].norm() + acc_signs[i] * float(rand()) / RAND_MAX * max_acc * delta_time;
        if (vel_new > max_vel)
        {
            vel_new = max_vel;
            acc_signs[i] = -1;
        }
        else if (vel_new < 0.01 * max_vel)
        {
            vel_new = 0.01 * max_vel;
            acc_signs[i] = 1;
        }
        // std::cout << "i: " << i << "  vel_new: " << vel_new << "\n";
        
        velocity = vel_new * velocities[i].normalized();
        pos = parts[i]->getTranslation();
        pos += velocity * delta_time;

        if (!isValid(pos, vel_new))
        {
            // std::cout << i << ". Computing new velocity.\n";
            velocity = fcl::Vector3f::Random(3);
            velocity.normalize();
            velocities[i] = vel_new * velocity;
            i--;
        }
        else
        {
            velocities[i] = velocity;
            parts[i]->setTranslation(pos);
            parts[i]->computeAABB();
            // std::cout << i << ". Obstacle pos: (" << pos.transpose() << ")\n";
        }
    }
    // std::cout << std::endl;
}
