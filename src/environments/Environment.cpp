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

env::Environment::~Environment() {}

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
        std::cout << "Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
    }
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
}

void env::Environment::removeCollisionObjects(int start_idx)
{
    for (auto idx = parts.end() - 1; idx >= parts.begin() + start_idx; idx--)
        parts.erase(idx);
}

void env::Environment::updateObstacles()
{
    fcl::Vector3f trans;
    for (int i = 0; i < parts.size(); i++)
    {
        trans = parts[i]->getTranslation();
        // if (trans(0) > 0)
        // {
            trans(0) -= 0.01;    // Move along x axis
            parts[i]->setTranslation(trans);
            parts[i]->computeAABB();
        // }
        // std::cout << "Obstacle range: (" << parts[i]->getAABB().min_.transpose() << ")\t(" << parts[i]->getAABB().max_.transpose() << ")\n";
    }
}
