//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>
#include "Planar2DOF.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"
#include <Eigen/Dense>

#include <glog/logging.h>

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::Planar2DOF::~Planar2DOF() {}

robots::Planar2DOF::Planar2DOF(std::string robot_desc, int num_DOFs)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
		throw std::runtime_error("Failed to construct kdl tree");

	urdf::Model model;
	if (!model.initFile(robot_desc))
    	throw std::runtime_error("Failed to parse urdf file");
	
	type = model.getName();
	LOG(INFO) << "Creating " << type;
	std::vector<urdf::LinkSharedPtr > links;
	model.getLinks(links);

	for (size_t i = 0; i < num_DOFs; ++i)
	{
		float lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		float upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits.emplace_back(std::vector<float>({lower, upper}));
	}

	for (size_t i = 0; i < links.size()-1; ++i)
	{
		
		if (links[i]->visual->geometry->type == urdf::Geometry::BOX)
		{
			auto box = (std::shared_ptr<urdf::Box>&) links[i]->visual->geometry;
			KDL::Vector origin(links[i]->visual->origin.position.x, 
							   links[i]->visual->origin.position.y,
							   links[i]->visual->origin.position.z);
			
			CollisionGeometryPtr fclBox(new fcl::Boxf(box->dim.x, box->dim.y, box->dim.z));
			//LOG(INFO) << "origin: " << origin << std::endl;
			
			init_poses.emplace_back(KDL::Frame(origin));
			parts.emplace_back(new fcl::CollisionObjectf(fclBox, fcl::Transform3f()));
			radii.emplace_back(box->dim.y / 2);
		}
	}
	//LOG(INFO) << "constructor----------------------\n";
	robot_tree.getChain("base_link", "tool", robot_chain);
	Eigen::VectorXf state = Eigen::VectorXf::Zero(num_DOFs);
	setState(std::make_shared<base::RealVectorSpaceState>(state));
}

std::shared_ptr<std::vector<KDL::Frame>> robots::Planar2DOF::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	setConfiguration(q);
	KDL::TreeFkSolverPos_recursive tree_fk_solver(robot_tree);
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk = std::make_shared<std::vector<KDL::Frame>>();
	robot_tree.getChain("base_link", "tool", robot_chain);
	KDL::JntArray joint_pos = KDL::JntArray(q->getDimensions());

	for (size_t i = 0; i < q->getDimensions(); ++i)
		joint_pos(i) = q->getCoord(i);
	
	for (size_t i = 0; i < robot_tree.getNrOfSegments(); ++i)
	{
		KDL::Frame cart_pos;
		bool kinematics_status = tree_fk_solver.JntToCart(joint_pos, cart_pos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
			frames_fk->emplace_back(cart_pos);
	}
	return frames_fk;
    
}

std::shared_ptr<base::State> robots::Planar2DOF::computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
	std::shared_ptr<base::State> q_init)
{
	// TODO (if needed)
}

std::shared_ptr<Eigen::MatrixXf> robots::Planar2DOF::computeSkeleton(std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames = computeForwardKinematics(q);
	std::shared_ptr<Eigen::MatrixXf> skeleton = std::make_shared<Eigen::MatrixXf>(3, getParts().size() + 1);
	for (int k = 0; k <= getParts().size(); k++)
		skeleton->col(k) << frames->at(k).p(0), frames->at(k).p(1), frames->at(k).p(2);
	
	return skeleton;
}

float robots::Planar2DOF::computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
									  std::shared_ptr<Eigen::MatrixXf> skeleton)
{
	// Assumes that number of links is equal to number of DOFs
	float d = 0;
	float r;
	for (int i = 0; i < getParts().size(); i++)
	{
		r = 0;
		for (int k = i+1; k <= getParts().size(); k++)
			r = std::max(r, (skeleton->col(k) - skeleton->col(i)).norm());
		
		d += r * std::abs(q2->getCoord(i) - q1->getCoord(i));
	}
	return fi / d;
}

void robots::Planar2DOF::setState(std::shared_ptr<base::State> q)
{
	KDL::JntArray joint_pos = KDL::JntArray(q->getDimensions());
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk = computeForwardKinematics(q);
	KDL::Frame tf;
	for (size_t i = 0; i < parts.size(); ++i)
	{
		tf = frames_fk->at(i) * init_poses[i];
		//LOG(INFO) << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
						
		//LOG(INFO) << "fcl\n";
		parts[i]->setTransform(KDL2fcl(tf));
		parts[i]->computeAABB(); 
		//LOG(INFO) << parts[i]->getAABB().min_ <<"\t;\t" << parts[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}
}

fcl::Vector3f robots::Planar2DOF::transformPoint(fcl::Vector3f& v, fcl::Transform3f t)
{
	fcl::Vector3f fclVec = t.translation();
	Eigen::Vector4f trans = Eigen::Vector4f(fclVec[0], fclVec[1], fclVec[2], 1);
	fcl::Matrix3f rot = t.rotation();
	Eigen::MatrixXf M = Eigen::MatrixXf::Identity(4,4);
	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 3; ++j)	
			M(i,j) = rot(i,j);
	for (size_t i = 0; i < 3; ++i)
		M(i,3) = fclVec[i];

	// LOG(INFO) << "obj TF:\n" << M << "\n\n"; 
	Eigen::Vector4f newVec = M * trans;
	return fcl::Vector3f(newVec(0), newVec(1), newVec(2));
}

fcl::Transform3f robots::Planar2DOF::KDL2fcl(const KDL::Frame &in)
{
	fcl::Transform3f out(fcl::Transform3f::Identity());
    double x, y, z, w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vector3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternionf q(w, x, y, z);
    out.linear() = q.matrix();
    out.translation() = t;
    return out;
}

KDL::Frame robots::Planar2DOF::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Matrix3f R = in.rotation();
	fcl::Quaternionf q(R);
    fcl::Vector3f t = in.translation();
    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    return f;
}

void robots::Planar2DOF::test(std::shared_ptr<env::Environment> env, std::shared_ptr<base::State> q)
{
	setState(q);
	std::shared_ptr<fcl::CollisionObject<float>> ob = env->getParts()[0];

	for (size_t i = 0; i < parts.size(); ++i)
	{
		fcl::DistanceRequest<float> request(true, 0.00, 0.00, fcl::GST_INDEP);
		fcl::DistanceResult<float> result;
		result.clear();
		fcl::distance(parts[i].get(), ob.get(), request, result);
		LOG(INFO) << "link " << i+1 << "\n" << parts[i]->getTransform().matrix();
		LOG(INFO) << "distance from " << i + 1 << ": " << result.min_distance << " p1: " << result.nearest_points[0].transpose()
				  << "\t p2: " << result.nearest_points[2].transpose();
	}
	//fcl::DefaultDistanceData<float> distance_data;
}