//
// Created by dinko on 07.02.22.
// Modified by nermin on 05.09.22.
//

#include "Planar2DOF.h"
#include "RealVectorSpaceState.h"

#include <urdf/model.h>
#include <glog/logging.h>

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::Planar2DOF::~Planar2DOF() {}

robots::Planar2DOF::Planar2DOF(const std::string &robot_desc, int num_DOFs_)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
		throw std::runtime_error("Failed to construct kdl tree");

	urdf::Model model;
	if (!model.initFile(robot_desc))
    	throw std::runtime_error("Failed to parse urdf file");
	
	type = model.getName();
	std::vector<urdf::LinkSharedPtr > links_;
	model.getLinks(links_);
	num_DOFs = num_DOFs_;

	for (int i = 0; i < num_DOFs; i++)
	{
		float lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		float upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits.emplace_back(lower, upper);
	}

	for (size_t i = 0; i < links_.size()-1; i++)
	{
		
		if (links_[i]->visual->geometry->type == urdf::Geometry::BOX)
		{
			#ifdef __GNUC__
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wstrict-aliasing"
			auto box = (std::shared_ptr<urdf::Box>&) links_[i]->visual->geometry;
				#pragma GCC diagnostic pop
			#endif
			
			KDL::Vector origin(links_[i]->visual->origin.position.x, 
							   links_[i]->visual->origin.position.y,
							   links_[i]->visual->origin.position.z);
			
			CollisionGeometryPtr fclBox(new fcl::Boxf(box->dim.x, box->dim.y, box->dim.z));
			// LOG(INFO) << "origin: " << origin << std::endl;
			
			init_poses.emplace_back(KDL::Frame(origin));
			links.emplace_back(new fcl::CollisionObjectf(fclBox, fcl::Transform3f()));
			capsules_radius.emplace_back(box->dim.y / 2);
		}
	}
	
	robot_tree.getChain("base_link", "tool", robot_chain);
	Eigen::VectorXf state = Eigen::VectorXf::Zero(num_DOFs);
	setState(std::make_shared<base::RealVectorSpaceState>(state));

	LOG(INFO) << type << " robot created.";
	// LOG(INFO) << "Constructor end ----------------------\n";
}

void robots::Planar2DOF::setState(const std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk = computeForwardKinematics(q);
	KDL::Frame tf;
	for (size_t i = 0; i < links.size(); i++)
	{
		tf = frames_fk->at(i) * init_poses[i];
		//LOG(INFO) << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
						
		//LOG(INFO) << "fcl\n";
		links[i]->setTransform(KDL2fcl(tf));
		links[i]->computeAABB(); 
		//LOG(INFO) << links[i]->getAABB().min_ <<"\t;\t" << links[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}
}

std::shared_ptr<std::vector<KDL::Frame>> robots::Planar2DOF::computeForwardKinematics(const std::shared_ptr<base::State> q)
{
	setConfiguration(q);
	KDL::TreeFkSolverPos_recursive tree_fk_solver(robot_tree);
	std::vector<KDL::Frame> frames_fk(num_DOFs);
	robot_tree.getChain("base_link", "tool", robot_chain);
	KDL::JntArray joint_pos = KDL::JntArray(num_DOFs);

	for (int i = 0; i < num_DOFs; i++)
		joint_pos(i) = q->getCoord(i);
	
	for (size_t i = 0; i < robot_tree.getNrOfSegments(); i++)
	{
		KDL::Frame cart_pos;
		bool kinematics_status = tree_fk_solver.JntToCart(joint_pos, cart_pos, robot_chain.getSegment(i).getName());
		if (kinematics_status)
			frames_fk[i] = cart_pos;
	}
	return std::make_shared<std::vector<KDL::Frame>>(frames_fk);
}

std::shared_ptr<base::State> robots::Planar2DOF::computeInverseKinematics([[maybe_unused]] const KDL::Rotation &R, [[maybe_unused]] const KDL::Vector &p, 
																		  [[maybe_unused]] const std::shared_ptr<base::State> q_init)
{
	// TODO (if needed)
	return nullptr;
}

std::shared_ptr<Eigen::MatrixXf> robots::Planar2DOF::computeSkeleton(const std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames = computeForwardKinematics(q);
	std::shared_ptr<Eigen::MatrixXf> skeleton = std::make_shared<Eigen::MatrixXf>(3, num_DOFs + 1);
	for (int k = 0; k <= num_DOFs; k++)
		skeleton->col(k) << frames->at(k).p(0), frames->at(k).p(1), frames->at(k).p(2);
	
	return skeleton;
}

// Compute step for moving from 'q1' towards 'q2' using ordinary bubble
float robots::Planar2DOF::computeStep(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float d_c, 
	float rho, const std::shared_ptr<Eigen::MatrixXf> skeleton)
{
	float d{0.f};
	float r{};
	for (size_t i = 0; i < links.size(); i++)
	{
		r = 0;
		for (size_t k = i+1; k <= links.size(); k++)
			r = std::max(r, (skeleton->col(k) - skeleton->col(i)).norm());
		
		d += r * std::abs(q2->getCoord(i) - q1->getCoord(i));
	}
	return (d_c - rho) / d;
}

// Compute step for moving from 'q1' towards 'q2' using expanded bubble
float robots::Planar2DOF::computeStep2(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, 
	const std::vector<float> &d_c_profile, const std::vector<float> &rho_profile, const std::shared_ptr<Eigen::MatrixXf> skeleton)
{
	[[maybe_unused]] float d{0.f};
	Eigen::VectorXf r = Eigen::VectorXf::Zero(links.size());
	for (size_t i = 0; i < links.size(); i++)
	{
		for (size_t k = i+1; k <= links.size(); k++)
			r(i) = std::max(r(i), (skeleton->col(k) - skeleton->col(i)).norm());
	}

	Eigen::VectorXf steps(links.size());
	for (size_t k = 0; k < links.size(); k++)
		steps(k) = (d_c_profile[k] - rho_profile[k]) / r.head(k+1).dot((q1->getCoord() - q2->getCoord()).head(k+1).cwiseAbs());

	return steps.minCoeff();
}

fcl::Vector3f robots::Planar2DOF::transformPoint([[maybe_unused]] fcl::Vector3f& v, fcl::Transform3f t)
{
	fcl::Vector3f fclVec = t.translation();
	Eigen::Vector4f trans = Eigen::Vector4f(fclVec[0], fclVec[1], fclVec[2], 1);
	fcl::Matrix3f rot = t.rotation();
	Eigen::MatrixXf M = Eigen::MatrixXf::Identity(4,4);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)	
			M(i,j) = rot(i,j);
	for (int i = 0; i < 3; i++)
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

void robots::Planar2DOF::test(const std::shared_ptr<env::Environment> env, const std::shared_ptr<base::State> q)
{
	setState(q);
	std::shared_ptr<fcl::CollisionObject<float>> ob = env->getCollObject(0);

	for (size_t i = 0; i < links.size(); i++)
	{
		fcl::DistanceRequest<float> request(true, 0.00, 0.00, fcl::GST_INDEP);
		fcl::DistanceResult<float> result;
		result.clear();
		fcl::distance(links[i].get(), ob.get(), request, result);
		LOG(INFO) << "link " << i+1 << "\n" << links[i]->getTransform().matrix();
		LOG(INFO) << "distance from " << i + 1 << ": " << result.min_distance << " p1: " << result.nearest_points[0].transpose()
				  << "\t p2: " << result.nearest_points[2].transpose();
	}
	// fcl::DefaultDistanceData<float> distance_data;
}