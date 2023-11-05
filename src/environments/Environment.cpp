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
        }
    }        
}

env::Environment::Environment(const fcl::Box<float> &box, const fcl::Transform3<float> &tf)
{
    CollisionGeometryPtr fclBox(new fcl::Box(box.side[0], box.side[1], box.side[2]));
	std::shared_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject(fclBox, tf));
    ob->computeAABB();
    parts.emplace_back(ob);
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
        std::cout << i << ". Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
    }
}

env::Environment::~Environment()
{
    parts.clear();
    velocities.clear();
}

void env::Environment::setParts(const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &parts_)
{
    parts.clear();
    parts = parts_;
    for (std::shared_ptr<fcl::CollisionObjectf> part : parts)
        part->computeAABB();
}

void env::Environment::addCollisionObject(const std::shared_ptr<fcl::CollisionObject<float>> ob) 
{
    ob->computeAABB();
    parts.emplace_back(ob);
    fcl::Vector3f vel = fcl::Vector3f::Random(3);
    vel.normalize();
    velocities.emplace_back(vel);
}

void env::Environment::removeCollisionObjects(int start_idx)
{
    for (int idx = parts.size()-1; idx >= start_idx; idx--)
    {
        parts.erase(parts.begin() + idx);
        velocities.erase(velocities.begin() + idx);
    }
}

// void env::Environment::updateEnvironment()
// {
//     fcl::Vector3f pos;
//     for (int i = 0; i < parts.size(); i++)
//     {
//         pos = parts[i]->getTranslation();
//         pos(0) -= max_vel;    // Move along x axis
//         parts[i]->setTranslation(pos);
//         parts[i]->computeAABB();
//         // std::cout << i << ". Obstacle range: (" << parts[i]->getAABB().min_.transpose() << ")\t(" << parts[i]->getAABB().max_.transpose() << ")\n";
//     }
// }

void env::Environment::updateEnvironment(float step)
{
    fcl::Vector3f pos, vel;
    for (int i = 0; i < parts.size(); i++)
    {
        pos = parts[i]->getTranslation();
        pos += step * max_vel * velocities[i];

        float tol = 0.245;  // Computed in order to make robot faster than obstacle
        if ((pos - WS_center).norm() > WS_radius ||                                         // Out of workspace
            pos.head(2).norm() < tol && pos.z() > -tol && pos.z() < WS_center.z() + tol)    // Robot base
        {
            // std::cout << i << ". Computing new velocity.\n";
            vel = fcl::Vector3f::Random(3);
            vel.normalize();
            velocities[i] = vel;
            i--;
        }
        else
        {
            parts[i]->setTranslation(pos);
            parts[i]->computeAABB();
            // std::cout << i << ". Obstacle pos: (" << pos.transpose() << ")\n";
        }
    }
    // std::cout << std::endl;
}
