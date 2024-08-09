#include "xArm6.h"

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::xArm6::~xArm6() {}

robots::xArm6::xArm6(const std::string &robot_desc, float gripper_length_, size_t ground_included_)
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
			CollisionGeometryPtr fcl_box(model);
			links.emplace_back(new fcl::CollisionObject(fcl_box, fcl::Transform3f()));
			init_poses.emplace_back(link_frame);
		}
	}
	
	self_collision_checking = true;
	gripper_length = gripper_length_;
	ground_included = ground_included_;
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

void robots::xArm6::setCapsulesRadius(const std::vector<float> &capsules_radius_)
{
	capsules_radius = capsules_radius_;
	capsules_radius_new = std::vector<float>(num_DOFs);
	for (size_t i = 0; i < num_DOFs-1; i++)
		capsules_radius_new[i] = std::max(capsules_radius[i], capsules_radius[i+1]);
	capsules_radius_new.back() = capsules_radius.back();
}

std::shared_ptr<std::vector<KDL::Frame>> robots::xArm6::computeForwardKinematics(const std::shared_ptr<base::State> q)
{
	setConfiguration(q);

	if (q->getFrames() != nullptr)		// It has been already computed!
		return q->getFrames();

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

	q->setFrames(std::make_shared<std::vector<KDL::Frame>>(frames_fk));
	return q->getFrames();
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
	std::shared_ptr<Eigen::MatrixXf> skeleton { std::make_shared<Eigen::MatrixXf>(3, num_DOFs + 1) };	// num_DOFs == getNumLinks() 
	skeleton->col(0) << 0, 0, 0;
	skeleton->col(1) << frames->at(1).p.x(), frames->at(1).p.y(), frames->at(1).p.z();
	skeleton->col(2) << frames->at(2).p.x(), frames->at(2).p.y(), frames->at(2).p.z();

	// KDL::Vector p3 { frames->at(2).p + frames->at(2).M.UnitX() * 0.0775 };
	KDL::Vector p3 { frames->at(3).p - frames->at(3).M.UnitZ() * 0.25 };
	skeleton->col(3) << p3.x(), p3.y(), p3.z();
	skeleton->col(4) << frames->at(4).p.x(), frames->at(4).p.y(), frames->at(4).p.z();

	KDL::Vector p5 { frames->at(4).p + frames->at(4).M.UnitX() * 0.076 };
	skeleton->col(5) << p5.x(), p5.y(), p5.z();
	skeleton->col(6) << frames->at(5).p.x(), frames->at(5).p.y(), frames->at(5).p.z();

    // Correct the last skeleton point regarding the attached gripper.
	KDL::Vector a { frames->back().M.UnitZ() };
	skeleton->col(6) -= 0.3 * gripper_length * Eigen::Vector3f(a.x(), a.y(), a.z());	// Line (*)
	
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
					R(i, j) = std::max(R(i, j-1), skeleton->col(j).head(2).norm() + capsules_radius_new[j-1]);
				break;

			case 3:	// Projection of 5. and 6. skeleton point to the link [3-4] is needed
				if (j >= 5)
				{
					Eigen::Vector3f AB { skeleton->col(4) - skeleton->col(3) };
					float t { (skeleton->col(j) - skeleton->col(3)).dot(AB) / AB.squaredNorm() };
					Eigen::Vector3f proj { skeleton->col(3) + t * AB };
					R(i, j) = std::max(R(i, j-1), (skeleton->col(j) - proj).norm() + capsules_radius_new[j-1]);
				}
				break;
			
			default:	// No projection is needed
				R(i, j) = std::max(R(i, j-1), (skeleton->col(j) - skeleton->col(i)).norm() + capsules_radius_new[j-1]);
				break;
			}
		}
	}

	q->setEnclosingRadii(std::make_shared<Eigen::MatrixXf>(R));
	return q->getEnclosingRadii();
}

/// @brief Check if there exists a self-collision when the robot moves from 'q1' to 'q2' following a straight line in C-space.
/// @param q1 Initial configuration
/// @param q2 Final configuration
/// @return True if there exists self-collision. Otherwise, return false.
/// If there exists self-collision, 'q2' will be modified such that it is equal to a final reached configuration 
/// from [q1,q2]-line that is collision-free.
/// @note Because of joint limits, only first two links can collide with the last two links for xArm6 robot.
bool robots::xArm6::checkSelfCollision(const std::shared_ptr<base::State> q1, std::shared_ptr<base::State> &q2)
{
	if (!self_collision_checking)
		return false;
	
	// auto time_start { std::chrono::steady_clock::now() };
	std::shared_ptr<base::State> q1_temp { std::make_shared<base::RealVectorSpaceState>(q1) };
	size_t num_iter { 0 };
	size_t max_num_iter { 5 };
	float phi2 {}, phi4 {}, phi5 {}, phi6 {};
	float step04 {}, step05 {}, step14 {}, step15 {}, step {};
	std::shared_ptr<Eigen::MatrixXf> R { nullptr };
	Eigen::VectorXf delta_q {};

	while (true)
	{
		R = computeEnclosingRadii(q1_temp);
		delta_q = (q2->getCoord() - q1_temp->getCoord()).cwiseAbs();
		phi2 = R->col(2).dot(delta_q); 	// Covered distance by skeleton point 2
		phi4 = R->col(4).dot(delta_q);	// Covered distance by skeleton point 4
		phi5 = R->col(5).dot(delta_q);	// Covered distance by skeleton point 5
		phi6 = R->col(6).dot(delta_q);	// Covered distance by skeleton point 6

		step04 = computeCapsulesDistance(q1_temp, 0, 4) / std::max(phi4, phi5);
		step05 = computeCapsulesDistance(q1_temp, 0, 5) / std::max(phi5, phi6);
		step14 = computeCapsulesDistance(q1_temp, 1, 4) / (phi2 + std::max(phi4, phi5));
		step15 = computeCapsulesDistance(q1_temp, 1, 5) / (phi2 + std::max(phi5, phi6));
		step = std::min(std::min(step04, step05), std::min(step14, step15));

		if (step > 1)
		{
			// std::cout << "Self-collision surely does not occur! \n";
			return false;
		}

		if (++num_iter == max_num_iter || step <= 0)
			break;

		if (step > 0)
			q1_temp = std::make_shared<base::RealVectorSpaceState>(q1_temp->getCoord() + step * (q2->getCoord() - q1_temp->getCoord()));
	}

	Eigen::VectorXf q1_temp_coord { q1_temp->getCoord() };
	Eigen::VectorXf q_new_coord {};
	std::shared_ptr<base::State> q_new { nullptr };
	base::State::Status status { base::State::Status::Advanced };
	const float D { (q2->getCoord() - q1_temp_coord).norm() };
	float dist { D };
	float step_crit {};
	std::vector<bool> skip_checking(4);
	
	while (status == base::State::Status::Advanced)
	{
		if (dist > RRTConnectConfig::EPS_STEP)
			q_new_coord = q1_temp_coord + (q2->getCoord() - q1_temp_coord) / dist * RRTConnectConfig::EPS_STEP;
		else
		{
			q_new_coord = q2->getCoord();
			status = base::State::Status::Reached;
		}

		dist = (q2->getCoord() - q_new_coord).norm();
		step_crit = 1 - dist / D;
		skip_checking = { step05 > step_crit, 
						  step15 > step_crit, 
						  step04 > step_crit, 
						  step14 > step_crit };
		
		q_new = std::make_shared<base::RealVectorSpaceState>(q_new_coord);
		// std::cout << "Checking self-collision for q_new: " << q_new << "\n";
		if (checkSelfCollision(q_new, skip_checking))
		{
			q2 = std::make_shared<base::RealVectorSpaceState>(q1_temp_coord);
			// std::cout << "Self-collision occured! Setting q2 to: " << q2 << "\n";
			return true;
		}

		q1_temp_coord = q_new_coord;
	}

	// std::cout << "No self-collision! \n";
	// std::cout << "Elapsed time: " 
	// 		  << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() / 1e3 << " [us]\n\n";
	return false;
}

/// @brief Check if there exists a self-collision when the xArm6 robot takes a configuration 'q'.
/// @param q A configuration to be considered.
/// @return True if there exists self-collision. Otherwise, return false.
/// @note Because of joint limits, only first two links can collide with the last two links for xArm6 robot.
bool robots::xArm6::checkSelfCollision(const std::shared_ptr<base::State> q)
{
	if (!self_collision_checking)
		return false;
		
	std::vector<bool> skip_checking(4, false);
	return checkSelfCollision(q, skip_checking);
}

/// @param skip_checking Determines whether collision checking between links: {0-5, 1-5, 0-4, 1-4} can be skipped.
bool robots::xArm6::checkSelfCollision(const std::shared_ptr<base::State> q, std::vector<bool> &skip_checking)
{
	if (!skip_checking[0] && computeCapsulesDistance(q, 0, 5) < 0 && checkRealSelfCollision(q, 0, 5))
	{
		// std::cout << "Self-collision between link 0 and link 5 exists! \n";
		return true;
	}

	if (!skip_checking[1] && computeCapsulesDistance(q, 1, 5) < 0 && checkRealSelfCollision(q, 1, 5))
	{
		// std::cout << "Self-collision between link 1 and link 5 exists! \n";
		return true;
	}
	
	if (!skip_checking[2] && computeCapsulesDistance(q, 0, 4) < 0 && checkRealSelfCollision(q, 0, 4))
	{
		// std::cout << "Self-collision between link 0 and link 4 exists! \n";
		return true;
	}

	if (!skip_checking[3] && computeCapsulesDistance(q, 1, 4) < 0 && checkRealSelfCollision(q, 1, 4))
	{
		// std::cout << "Self-collision between link 1 and link 4 exists! \n";
		return true;
	}

	return false;
}

/// @brief Compute distance between two corresponding capsules of 'link1_idx'-th and 'link2_idx'-th links for xArm6 robot.
/// @param q Configuration of the robot
/// @param link1_idx Index of the first link
/// @param link2_idx Index of the second link
float robots::xArm6::computeCapsulesDistance(const std::shared_ptr<base::State> q, size_t link1_idx, size_t link2_idx)
{
	std::shared_ptr<Eigen::MatrixXf> skeleton { computeSkeleton(q) };

	return std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg
		   (skeleton->col(link1_idx), skeleton->col(link1_idx+1), skeleton->col(link2_idx), skeleton->col(link2_idx+1))) 
		   - getCapsuleRadius(link1_idx) - getCapsuleRadius(link2_idx);
}

bool robots::xArm6::checkRealSelfCollision(const std::shared_ptr<base::State> q, size_t link1_idx, size_t link2_idx)
{
	// std::cout << "Checking real self-collision between link " << link1_idx << " and link " << link2_idx << "... \n";
	setState(q);

	if (checkCollisionFCL(links[link1_idx], links[link2_idx]))
		return true;

	if (link2_idx == 5 && gripper_length > 0) 	// Self-collision with the gripper needs to be considered particularly.
	{
		std::shared_ptr<Eigen::MatrixXf> skeleton { computeSkeleton(q) };		
		Eigen::Vector3f A { skeleton->col(6) - 0.7 * gripper_length * (skeleton->col(6) - skeleton->col(5)).normalized() };	// see Line (*)
        Eigen::Vector3f B { skeleton->col(6) };
		fcl::Vector3f trans(
			(A(0) + B(0)) / 2, 
			(A(1) + B(1)) / 2, 
			(A(2) + B(2)) / 2);
		fcl::Matrix3f rot {};
		rot.col(2) = B - A;
		rot.col(0) = (Eigen::Vector3f::UnitZ().cross(rot.col(2))).normalized();
		rot.col(1) = (rot.col(2).cross(rot.col(0)).normalized());

		// Gripper is approximated by an enclosing capsule
		CollisionGeometryPtr gripper(new fcl::Capsulef(getCapsuleRadius(num_DOFs-1), 0.7 * gripper_length));
		std::unique_ptr<fcl::CollisionObjectf> gripper_fcl(new fcl::CollisionObjectf(gripper, fcl::Transform3f()));
		gripper_fcl->setTranslation(trans);
		gripper_fcl->setRotation(rot);

		if (checkCollisionFCL(links[link1_idx], gripper_fcl))
			return true;

		// Alternative conservative checking
		// if (std::get<0>(base::RealVectorSpace::distanceLineSegToLineSeg(skeleton->col(link1_idx), skeleton->col(link1_idx+1), A, B)) 
		// 	- getCapsuleRadius(link1_idx) - getCapsuleRadius(link2_idx) < 0)
		// 	return true;
	}

	return false;
}

bool robots::xArm6::checkCollisionFCL(const std::unique_ptr<fcl::CollisionObjectf> &obj1, const std::unique_ptr<fcl::CollisionObjectf> &obj2)
{
	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_obj1 { std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>() };
	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_obj2 { std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>() };
	fcl::DefaultCollisionData<float> collision_data {};

	collision_manager_obj1->clear();
	collision_manager_obj2->clear();
	collision_manager_obj1->registerObject(obj1.get());
	collision_manager_obj2->registerObject(obj2.get());
	collision_manager_obj1->setup();
	collision_manager_obj2->setup();
	collision_manager_obj1->collide(collision_manager_obj2.get(), &collision_data, fcl::DefaultCollisionFunction);
	if (collision_data.result.isCollision())
		return true;

	return false;
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
    f.p = KDL::Vector(t[0], t[1], t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
    return f;
}

void robots::xArm6::test()
{
	CollisionGeometryPtr fcl_box(new fcl::Box<float>(0.5, 1.0, 3.0));
	fcl::Transform3f tf {}; 
	tf.translation() = fcl::Vector3f(1, 1, 1.5);
	std::unique_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject<float>(fcl_box, tf));
	for (size_t i = 0; i < links.size(); i++)
	{
		fcl::DistanceRequest<float> request {};
		fcl::DistanceResult<float> result {};
		fcl::distance(links[i].get(), ob.get(), request, result);
		LOG(INFO) << links[i]->getAABB().min_ <<"\t;\t" << links[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		LOG(INFO) << "Distance from " << i+1 << ": " << result.min_distance << std::endl;
	}
}
