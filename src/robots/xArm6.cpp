//
// Created by dinko on 07.02.22.
// Modified by nermin on 05.09.22.
//

#include "xArm6.h"
#include "RealVectorSpace.h"

#include <urdf/model.h>
#include <glog/logging.h>
#include <stl_reader.h>

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::xArm6::~xArm6() {}

robots::xArm6::xArm6(const std::string &robot_desc, float gripper_length_, bool table_included_)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
		throw std::runtime_error("Failed to construct kdl tree");

	urdf::Model model {};
	if (!model.initFile(robot_desc))
    	throw std::runtime_error("Failed to parse urdf file");
	
	const size_t last_slash_idx = robot_desc.rfind('/');
	std::string urdf_root_path = robot_desc.substr(0, last_slash_idx) + "/";
	type = model.getName();
	std::vector<urdf::LinkSharedPtr> links_;
	model.getLinks(links_);
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	num_DOFs = robot_chain.getNrOfJoints();
	float lower { 0 };
	float upper { 0 };

	for (size_t i = 0; i < num_DOFs; i++)
	{
		lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits.emplace_back(std::pair<float, float>(lower, upper));

		// LOG(INFO) << links_[i]->name << "\t" << links_[i]->collision->geometry->type;
		urdf::Pose pose = model.getJoint("joint"+std::to_string(i+1))->parent_to_joint_origin_transform;
		KDL::Vector pos(pose.position.x, pose.position.y, pose.position.z);
		double roll { 0 }, pitch { 0 }, yaw { 0 };
		KDL::Frame link_frame;
		link_frame.p = pos;
		link_frame.M = link_frame.M.RPY(roll, pitch, yaw);
		// LOG(INFO) << link_frame;
		if (links_[i]->collision->geometry->type == urdf::Geometry::MESH)
		{
			fcl::Vector3f p[3];
			fcl::BVHModel<fcl::OBBRSS<float>>* model { new fcl::BVHModel<fcl::OBBRSS<float>> };
    		model->beginModel();
			const auto mesh_ptr { dynamic_cast<const urdf::Mesh*>(links_[i]->collision->geometry.get()) };			
			stl_reader::StlMesh <float, unsigned int> mesh (urdf_root_path + mesh_ptr->filename);
			for (size_t j = 0; j < mesh.num_tris(); j++)
			{
				for (size_t icorner = 0; icorner < 3; icorner++) 
				{
					const float* c { mesh.vrt_coords (mesh.tri_corner_ind (j, icorner)) };
					// LOG(INFO) << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
					p[icorner] = fcl::Vector3f(c[0], c[1], c[2]); 
				}
				// LOG(INFO) << "+++++++++++++++++++++++++++++";
				model->addTriangle(p[0], p[1], p[2]);
			}
			model->endModel();
			CollisionGeometryPtr fclBox(model);
			links.emplace_back(new fcl::CollisionObject(fclBox, fcl::Transform3f()));
			init_poses.emplace_back(link_frame);
		}
	}
	
	gripper_length = gripper_length_;
	table_included = table_included_;
	if (table_included)
		type += "_with_table";

	Eigen::VectorXf state { Eigen::VectorXf::Zero(num_DOFs) };
	if (gripper_length > 0)
		state << 0, 0, 0, M_PI, M_PI_2, 0; 	// Starting configuration in case the gripper is attached

	setState(std::make_shared<base::RealVectorSpaceState>(state));

	LOG(INFO) << type << " robot created.";
	// LOG(INFO) << "Constructor end ----------------------\n";
}

void robots::xArm6::setState(const std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk { computeForwardKinematics(q) };
	KDL::Frame tf {};
	for (size_t i = 0; i < num_DOFs; i++)
	{
		tf = frames_fk->at(i);
		// LOG(INFO) << "kdl\n" << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
		// fcl::Transform3f tf_fcl = KDL2fcl(tf);
		// LOG(INFO) << "fcl\n" << tf_fcl.translation().transpose() << "\t;\n" << tf_fcl.linear() << "\n..................................\n";
		
		links[i]->setTransform(KDL2fcl(tf));
		links[i]->computeAABB(); 
		// LOG(INFO) << links[i]->getAABB().min_ <<"\t;" << std::endl;
		// LOG(INFO) << links[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		// LOG(INFO) << (links[i]->getAABB().max_ - links[i]->getAABB().min_).transpose() <<"\t;" << std::endl;
		// LOG(INFO) << ((links[i]->getAABB().min_ + links[i]->getAABB().max_) / 2).transpose() << std::endl << "*******************" << std::endl;
	}
}

std::shared_ptr<std::vector<KDL::Frame>> robots::xArm6::computeForwardKinematics(const std::shared_ptr<base::State> q)
{
	setConfiguration(q);
	KDL::TreeFkSolverPos_recursive tree_fk_solver(robot_tree);
	std::vector<KDL::Frame> frames_fk(num_DOFs);
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::JntArray joint_pos { KDL::JntArray(num_DOFs) };

	for (size_t i = 0; i < num_DOFs; i++)
		joint_pos(i) = q->getCoord(i);

	for (size_t i = 0; i < num_DOFs; i++)
	{
		KDL::Frame cart_pos {};
		tree_fk_solver.JntToCart(joint_pos, cart_pos, robot_chain.getSegment(i).getName());
		frames_fk[i] = cart_pos;
		// std::cout << "Frame R" << i << ": " << frames_fk[i].M << "\n";
		// std::cout << "Frame p" << i << ": " << frames_fk[i].p << "\n";
	}
	frames_fk.back().p += gripper_length * frames_fk.back().M.UnitZ();

	return std::make_shared<std::vector<KDL::Frame>>(frames_fk);
}

std::shared_ptr<base::State> robots::xArm6::computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
																	 const std::shared_ptr<base::State> q_init)
{
	KDL::Vector p_new { p - gripper_length * R.UnitZ() };
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
	KDL::ChainIkSolverVel_pinv ik_solver(robot_chain);
	KDL::ChainIkSolverPos_NR ik_solver_pos(robot_chain, fk_solver, ik_solver, 1000, 1e-5);

	KDL::JntArray q_in(num_DOFs);
	KDL::JntArray q_out(num_DOFs);
	KDL::Frame goal_frame(R, p_new);
	Eigen::VectorXf q_result(num_DOFs);

	float error { INFINITY };
	size_t num { 0 };
	while (error > 1e-5)
	{
		if (q_init == nullptr)
		{
			Eigen::VectorXf rand { Eigen::VectorXf::Random(num_DOFs) };
			for (size_t i = 0; i < num_DOFs; i++)
				q_in.data(i) = ((limits[i].second - limits[i].first) * rand(i) + limits[i].first + limits[i].second) / 2;
		}
		else
		{
			for (size_t i = 0; i < num_DOFs; i++)
				q_in.data(i) = q_init->getCoord(i);
		}

		ik_solver_pos.CartToJnt(q_in, goal_frame, q_out);
		error = ik_solver_pos.getError();
		// std::cout << "error: " << error << std::endl;
		for (size_t i = 0; i < num_DOFs; i++)
		{
			// Set the angle between -PI and PI
			q_result(i) = q_out.data(i) - int(q_out.data(i) / (2*M_PI)) * 2*M_PI;
			if (q_result(i) < -M_PI)
				q_result(i) += 2*M_PI;
			else if (q_result(i) > M_PI)
				q_result(i) -= 2*M_PI;

			if (q_result(i) > limits[i].second)
			{
				q_result(i) -= 2*M_PI;
				if (q_result(i) < limits[i].first)
				{
					error = INFINITY; 	// Try to compute IK again.
					break;
				}
			}
			else if (q_result(i) < limits[i].first)
			{
				q_result(i) += 2*M_PI;
				if (q_result(i) > limits[i].second)
				{
					error = INFINITY;	// Try to compute IK again.
					break;
				}
			}
		}

		// std::cout << "num: " << num << std::endl;
		if (num++ > 1000)
		{
			std::cout << "Unable to compute inverse kinematics for the given input frame! \n";
			return nullptr;
		}
	}

	return std::make_shared<base::RealVectorSpaceState>(q_result);
}

std::shared_ptr<Eigen::MatrixXf> robots::xArm6::computeSkeleton(const std::shared_ptr<base::State> q)
{
	if (q->getSkeleton() != nullptr)	// It has been already computed!
		return q->getSkeleton();
	
	std::shared_ptr<std::vector<KDL::Frame>> frames { computeForwardKinematics(q) };
	std::shared_ptr<Eigen::MatrixXf> skeleton { std::make_shared<Eigen::MatrixXf>(3, num_DOFs + 1) };
	skeleton->col(0) << 0, 0, 0;
	skeleton->col(1) << frames->at(1).p(0), frames->at(1).p(1), frames->at(1).p(2);
	skeleton->col(2) << frames->at(2).p(0), frames->at(2).p(1), frames->at(2).p(2);

	// KDL::Vector p3 { frames->at(2).p + frames->at(2).M.UnitX() * 0.0775 };
	KDL::Vector p3 { frames->at(3).p - frames->at(3).M.UnitZ() * 0.25 };
	skeleton->col(3) << p3(0), p3(1), p3(2);
	skeleton->col(4) << frames->at(4).p(0), frames->at(4).p(1), frames->at(4).p(2);

	KDL::Vector p5 { frames->at(4).p + frames->at(4).M.UnitX() * 0.076 };
	skeleton->col(5) << p5(0), p5(1), p5(2);
	skeleton->col(6) << frames->at(5).p(0), frames->at(5).p(1), frames->at(5).p(2);

    // Correct the last skeleton point regarding the attached gripper.
	KDL::Vector a { frames->back().M.UnitZ() };
	skeleton->col(6) -= 0.3 * gripper_length * Eigen::Vector3f(a.x(), a.y(), a.z());
	
	q->setSkeleton(skeleton);
	return skeleton;
}

std::shared_ptr<Eigen::MatrixXf> robots::xArm6::computeEnclosingRadii(const std::shared_ptr<base::State> q)
{
	if (q->getEnclosingRadii() != nullptr)	// It has been already computed!
		return q->getEnclosingRadii();

	std::shared_ptr<Eigen::MatrixXf> skeleton { computeSkeleton(q) };
	Eigen::MatrixXf R { Eigen::MatrixXf::Zero(num_DOFs, num_DOFs+1) };

	for (size_t i = 0; i < num_DOFs-1; i++) 		// Starting point on skeleton
	{
		for (size_t j = i+1; j <= num_DOFs; j++)	// Final point on skeleton
		{
			switch (i)
			{
			case 0:	// Special case when all frame origins are projected on {x,y} plane
				if (j >= 2)
					R(i, j) = skeleton->col(j).head(2).norm() + capsules_radius[j-1];
				break;

			case 3:	// Projection of 5. and 6. skeleton point to the link [3-4] is needed
				if (j >= 5)
				{
					Eigen::Vector3f AB { skeleton->col(4) - skeleton->col(3) };
					float t { (skeleton->col(j) - skeleton->col(3)).dot(AB) / AB.squaredNorm() };
					Eigen::Vector3f proj { skeleton->col(3) + t * AB };
					R(i, j) = (skeleton->col(j) - proj).norm() + capsules_radius[j-1];
				}
				break;
			
			default:	// No projection is needed
				R(i, j) = (skeleton->col(j) - skeleton->col(i)).norm() + capsules_radius[j-1];
				break;
			}
		}
	}

	q->setEnclosingRadii(std::make_shared<Eigen::MatrixXf>(R));
	return q->getEnclosingRadii();
}

/// @brief Check if there exists a self-collision when robot moves from 'q1' to 'q2' following a straight line in C-space.
/// @param q1 Initial configuration
/// @param q2 Final configuration
/// @return True if there exists self-collision. Otherwise, return false.
/// If there exists self-collision, 'q2' will be modified such that it is equal to a final reached configuration from [q1,q2]-line that is collision-free.
/// @note Because of joint limits, only first two links can collide with the last two links for xArm6 robot.
bool robots::xArm6::checkSelfCollision(const std::shared_ptr<base::State> q1, std::shared_ptr<base::State> &q2)
{
	std::shared_ptr<Eigen::MatrixXf> skeleton1 { computeSkeleton(q1) };
	std::shared_ptr<Eigen::MatrixXf> skeleton2 { computeSkeleton(q2) }; 	// Mozda i ne treba...
	
	// Paziti na gripper. On se ne provjerava kroz FCL!!!

	auto time_start = std::chrono::steady_clock::now();

	float d_04 { std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg 	// distance between link0 and link4
				 (skeleton1->col(0), skeleton1->col(1), skeleton1->col(4), skeleton1->col(5))) 
				 - getCapsuleRadius(0) - getCapsuleRadius(4) };
	float d_05 { std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg	// distance between link0 and link5
				 (skeleton1->col(0), skeleton1->col(1), skeleton1->col(5), skeleton1->col(6))) 
				 - getCapsuleRadius(0) - getCapsuleRadius(5) };
	float d_14 { std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg	// distance between link1 and link4
				 (skeleton1->col(1), skeleton1->col(2), skeleton1->col(4), skeleton1->col(5))) 
				 - getCapsuleRadius(1) - getCapsuleRadius(4) };
	float d_15 { std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg	// distance between link1 and link5
				 (skeleton1->col(1), skeleton1->col(2), skeleton1->col(5), skeleton1->col(6))) 
				 - getCapsuleRadius(1) - getCapsuleRadius(5) };
	float d_c_min { std::min(std::min(d_04, d_05), std::min(d_14, d_15)) };

	std::vector<float> rho(7);
	for (size_t k = 1; k <= getNumLinks(); k++)
	{
		rho[k] = (skeleton1->col(k) - skeleton2->col(k)).norm();
		// std::cout << "k: " << k << "\t rho_k: " << rho[k] << "\n";
	}

	float phi { std::max(rho[1], rho[2]) + std::max(std::max(rho[4], rho[5]), rho[6]) };
	std::cout << "Covered distance: " << phi << "\n";
	std::cout << "Elapsed time: " 
			  << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() / 1e3 << " [us]\n";

	if (phi > d_c_min)
	{
		std::cout << "******************* Maybe self-collision ******************* \n";
		setState(q1);
		std::cout << "Collision 0-1 with 4-5: " << checkRealSelfCollision(0, 4) << "\n";
		std::cout << "Collision 0-1 with 5-6: " << checkRealSelfCollision(0, 5) << "\n";
		std::cout << "Collision 1-2 with 4-5: " << checkRealSelfCollision(1, 4) << "\n";
		std::cout << "Collision 1-2 with 5-6: " << checkRealSelfCollision(1, 5) << "\n";
		std::cout << "--------------\n";
	}
	
	return false;
}

bool robots::xArm6::checkRealSelfCollision(size_t link1_idx, size_t link2_idx)
{
	auto time_start = std::chrono::steady_clock::now();
	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_link1 { std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>() };
	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_link2 { std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>() };
	fcl::DefaultCollisionData<float> collision_data {};

	collision_manager_link1->clear();
	collision_manager_link2->clear();
	collision_manager_link1->registerObject(links[link1_idx].get());
	collision_manager_link2->registerObject(links[link2_idx].get());
	collision_manager_link1->setup();
	collision_manager_link2->setup();
	collision_manager_link1->collide(collision_manager_link2.get(), &collision_data, fcl::DefaultCollisionFunction);

	std::cout << "Elapsed time for real self-collision: " 
			  << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() / 1e3 << " [us]\n";

	return collision_data.result.isCollision();
}

fcl::Transform3f robots::xArm6::KDL2fcl(const KDL::Frame &in)
{
	fcl::Transform3f out {};
    double x { 0 }, y { 0 }, z { 0 }, w { 0 };
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vector3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternionf q(w, x, y, z);
    out.linear() = q.matrix();
    out.translation() = t;

    return out;
}

KDL::Frame robots::xArm6::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Matrix3f R { in.rotation() };
	fcl::Quaternionf q(R);
    fcl::Vector3f t { in.translation() };

    KDL::Frame f {};
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
    return f;
}

void robots::xArm6::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box<float>(0.5, 1.0, 3.0));
	fcl::Transform3f tf {}; 
	tf.translation() = fcl::Vector3f(1, 1, 1.5);
	std::unique_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject<float>(fclBox, tf));
	for (size_t i = 0; i < links.size(); i++)
	{
		fcl::DistanceRequest<float> request {};
		fcl::DistanceResult<float> result {};
		fcl::distance(links[i].get(), ob.get(), request, result);
		LOG(INFO) << links[i]->getAABB().min_ <<"\t;\t" << links[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		LOG(INFO) << "Distance from " << i+1 << ": " << result.min_distance << std::endl;
	}
}
