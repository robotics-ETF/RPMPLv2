//
// Created by dinko on 07.02.22.
// Modified by nermin on 05.09.22.
//

#include "xArm6.h"
#include "RealVectorSpaceState.h"

#include <urdf/model.h>
#include <glog/logging.h>
#include <stl_reader.h>

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::xArm6::~xArm6() {}

robots::xArm6::xArm6(std::string robot_desc, float gripper_length_, bool table_included_)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
		throw std::runtime_error("Failed to construct kdl tree");

	urdf::Model model;
	if (!model.initFile(robot_desc))
    	throw std::runtime_error("Failed to parse urdf file");
	
	const size_t last_slash_idx = robot_desc.rfind('/');
	std::string urdf_root_path = robot_desc.substr(0, last_slash_idx) + "/";
	type = model.getName();
	std::vector<urdf::LinkSharedPtr> links;
	model.getLinks(links);
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	num_DOFs = robot_chain.getNrOfJoints();

	for (size_t i = 0; i < num_DOFs; ++i)
	{
		float lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		float upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits.emplace_back(std::vector<float>({lower, upper}));

		// LOG(INFO) << links[i]->name << "\t" << links[i]->collision->geometry->type;
		urdf::Pose pose = model.getJoint("joint"+std::to_string(i+1))->parent_to_joint_origin_transform;
		KDL::Vector pos(pose.position.x, pose.position.y, pose.position.z);
		double roll, pitch, yaw;
		KDL::Frame link_frame;
		link_frame.p = pos;
		link_frame.M = link_frame.M.RPY(roll, pitch, yaw);
		// LOG(INFO) << link_frame;
		if (links[i]->collision->geometry->type == urdf::Geometry::MESH)
		{
			fcl::Vector3f p[3];
			fcl::BVHModel<fcl::OBBRSS<float>>* model = new fcl::BVHModel<fcl::OBBRSS<float>>;
    		model->beginModel();
			const auto mesh_ptr = dynamic_cast<const urdf::Mesh*>(links[i]->collision->geometry.get());			
			stl_reader::StlMesh <float, unsigned int> mesh (urdf_root_path + mesh_ptr->filename);
			for (int j = 0; j < mesh.num_tris(); ++j)
			{
				for (size_t icorner = 0; icorner < 3; ++icorner) 
				{
					const float* c = mesh.vrt_coords (mesh.tri_corner_ind (j, icorner));
					// LOG(INFO) << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
					p[icorner] = fcl::Vector3f(c[0], c[1], c[2]); 
				}
				// LOG(INFO) << "+++++++++++++++++++++++++++++";
				model->addTriangle(p[0], p[1], p[2]);
			}
			model->endModel();
			CollisionGeometryPtr fclBox(model);
			parts.emplace_back(new fcl::CollisionObject(fclBox, fcl::Transform3f()));
			init_poses.emplace_back(link_frame);
		}
	}
	
	gripper_length = gripper_length_;
	table_included = table_included_;
	if (table_included)
		type += "_with_table";

	Eigen::VectorXf state = Eigen::VectorXf::Zero(num_DOFs);
	if (gripper_length > 0)
		state << 0, 0, 0, M_PI, M_PI_2, 0; 	// Starting configuration in case the gripper is attached
	setState(std::make_shared<base::RealVectorSpaceState>(state));

	LOG(INFO) << type << " robot created.";
	// LOG(INFO) << "Constructor end ----------------------\n";
}

void robots::xArm6::setState(std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk = computeForwardKinematics(q);
	KDL::Frame tf;
	for (size_t i = 0; i < parts.size(); ++i)
	{
		tf = frames_fk->at(i);
		// LOG(INFO) << "kdl\n" << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
		// fcl::Transform3f tf_fcl = KDL2fcl(tf);
		// LOG(INFO) << "fcl\n" << tf_fcl.translation().transpose() << "\t;\n" << tf_fcl.linear() << "\n..................................\n";
		
		parts[i]->setTransform(KDL2fcl(tf));
		parts[i]->computeAABB(); 
		// LOG(INFO) << parts[i]->getAABB().min_ <<"\t;" << std::endl;
		// LOG(INFO) << parts[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		// LOG(INFO) << (parts[i]->getAABB().max_ - parts[i]->getAABB().min_).transpose() <<"\t;" << std::endl;
		// LOG(INFO) << ((parts[i]->getAABB().min_ + parts[i]->getAABB().max_) / 2).transpose() << std::endl << "*******************" << std::endl;
	}
}

std::shared_ptr<std::vector<KDL::Frame>> robots::xArm6::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	setConfiguration(q);
	KDL::TreeFkSolverPos_recursive tree_fk_solver(robot_tree);
	std::shared_ptr<std::vector<KDL::Frame>> frames_fk = std::make_shared<std::vector<KDL::Frame>>();
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::JntArray joint_pos = KDL::JntArray(num_DOFs);

	for (size_t i = 0; i < num_DOFs; ++i)
		joint_pos(i) = q->getCoord(i);

	for (size_t i = 0; i < num_DOFs; ++i)
	{
		KDL::Frame cart_pos;
		bool kinematics_status = tree_fk_solver.JntToCart(joint_pos, cart_pos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
		{
			frames_fk->emplace_back(cart_pos);
			// std::cout << "Frame R" << i << ": " << frames_fk->at(i).M << std::endl;
			// std::cout << "Frame p" << i << ": " << frames_fk->at(i).p << std::endl;
		}
	}
	
	frames_fk->back().p += gripper_length * frames_fk->back().M.UnitZ();

	return frames_fk;
}

std::shared_ptr<base::State> robots::xArm6::computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
																	 std::shared_ptr<base::State> q_init)
{
	KDL::Vector p_new = p - gripper_length * R.UnitZ();
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
	KDL::ChainIkSolverVel_pinv ik_solver(robot_chain);
	KDL::ChainIkSolverPos_NR ik_solver_pos(robot_chain, fk_solver, ik_solver, 1000, 1e-5);

	KDL::JntArray q_in(num_DOFs);
	KDL::JntArray q_out(num_DOFs);
	KDL::Frame goal_frame(R, p_new);
	Eigen::VectorXf q_result(num_DOFs);

	srand((unsigned int) time(0));
	float error = INFINITY;
	int num = 0;
	while (error > 1e-5)
	{
		if (q_init == nullptr)
		{
			Eigen::VectorXf rand = Eigen::VectorXf::Random(num_DOFs);
			for (size_t i = 0; i < num_DOFs; i++)
				q_in.data(i) = ((limits[i][1] - limits[i][0]) * rand(i) + limits[i][0] + limits[i][1]) / 2;
		}
		else
		{
			for (size_t i = 0; i < num_DOFs; i++)
				q_in.data(i) = q_init->getCoord(i);
		}

		ik_solver_pos.CartToJnt(q_in, goal_frame, q_out);
		error = ik_solver_pos.getError();
		// std::cout << "error: " << error << std::endl;
		for (int i = 0; i < num_DOFs; i++)
		{
			// Set the angle between -PI and PI
			q_result(i) = q_out.data(i) - int(q_out.data(i) / (2*M_PI)) * 2*M_PI;
			if (q_result(i) < -M_PI)
				q_result(i) += 2*M_PI;
			else if (q_result(i) > M_PI)
				q_result(i) -= 2*M_PI;

			if (q_result(i) > limits[i][1])
			{
				q_result(i) -= 2*M_PI;
				if (q_result(i) < limits[i][0])
				{
					error = INFINITY; 	// Try to compute IK again.
					break;
				}
			}
			else if (q_result(i) < limits[i][0])
			{
				q_result(i) += 2*M_PI;
				if (q_result(i) > limits[i][1])
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

std::shared_ptr<Eigen::MatrixXf> robots::xArm6::computeSkeleton(std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames = computeForwardKinematics(q);
	std::shared_ptr<Eigen::MatrixXf> skeleton = std::make_shared<Eigen::MatrixXf>(3, parts.size() + 1);
	skeleton->col(0) << 0, 0, 0;
	skeleton->col(1) << frames->at(1).p(0), frames->at(1).p(1), frames->at(1).p(2);
	skeleton->col(2) << frames->at(2).p(0), frames->at(2).p(1), frames->at(2).p(2);

	// KDL::Vector p3 = frames->at(2).p + frames->at(2).M.UnitX() * 0.0775;
	KDL::Vector p3 = frames->at(3).p - frames->at(3).M.UnitZ() * 0.25;
	skeleton->col(3) << p3(0), p3(1), p3(2);
	skeleton->col(4) << frames->at(4).p(0), frames->at(4).p(1), frames->at(4).p(2);

	KDL::Vector p5 = frames->at(4).p + frames->at(4).M.UnitX() * 0.076;
	skeleton->col(5) << p5(0), p5(1), p5(2);
	skeleton->col(6) << frames->at(5).p(0), frames->at(5).p(1), frames->at(5).p(2);

    // Correct the last skeleton point regarding the attached gripper.
	KDL::Vector a = frames->back().M.UnitZ();
	skeleton->col(6) -= 0.3 * gripper_length * Eigen::Vector3f(a.x(), a.y(), a.z());
	
	return skeleton;
}

// Compute step for moving from 'q1' towards 'q2' using ordinary bubble
float robots::xArm6::computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float d_c, float rho, 
								 std::shared_ptr<Eigen::MatrixXf> skeleton)
{
	Eigen::VectorXf r(parts.size()); 	// For robot xArm6, parts.size() = 6
	r(0) = getEnclosingRadius(skeleton, 2, -2);
	r(1) = getEnclosingRadius(skeleton, 2, -1);
	r(2) = getEnclosingRadius(skeleton, 3, -1);
	r(3) = getEnclosingRadius(skeleton, 5, 3);
	r(4) = getEnclosingRadius(skeleton, 5, -1);
	r(5) = 0;

	return (d_c - rho) / r.dot((q1->getCoord() - q2->getCoord()).cwiseAbs()); 	// 'd_c - rho' is the remaining path length in W-space
}

// Compute step for moving from 'q1' towards 'q2' using expanded bubble
float robots::xArm6::computeStep2(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, const std::vector<float> &d_c_profile,
						   		  const std::vector<float> &rho_profile, std::shared_ptr<Eigen::MatrixXf> skeleton)
{
	Eigen::VectorXf r(parts.size()); 	// For robot xArm6, parts.size() = 6
	r(0) = getEnclosingRadius(skeleton, 2, -2);
	r(1) = getEnclosingRadius(skeleton, 2, -1);
	r(2) = getEnclosingRadius(skeleton, 3, -1);
	r(3) = getEnclosingRadius(skeleton, 5, 3);
	r(4) = getEnclosingRadius(skeleton, 5, -1);
	r(5) = 0;

	Eigen::VectorXf steps(parts.size());
	for (int k = 0; k < parts.size(); k++)
		steps(k) = (d_c_profile[k] - rho_profile[k]) / r.head(k+1).dot((q1->getCoord() - q2->getCoord()).head(k+1).cwiseAbs());

	return steps.minCoeff();
}

float robots::xArm6::getEnclosingRadius(std::shared_ptr<Eigen::MatrixXf> skeleton, int j_start, int j_proj)
{
	float r = 0;
	if (j_proj == -2)	// Special case when all frame origins starting from j_start are projected on {x,y} plane
	{
		for (int j = j_start; j < skeleton->cols(); j++)
			r = std::max(r, skeleton->col(j).head(2).norm() + capsules_radius[j-1]);
	}
	else if (j_proj == -1) 	// No projection
	{
		for (int j = j_start; j < skeleton->cols(); j++)
			r = std::max(r, (skeleton->col(j) - skeleton->col(j_start-1)).norm() + capsules_radius[j-1]);
	}
	else	// Projection of all frame origins starting from j_start to the link (j_proj, j_proj+1) is needed
	{
		float t;
		Eigen::Vector3f A = skeleton->col(j_proj);
		Eigen::Vector3f B = skeleton->col(j_proj+1);
		Eigen::Vector3f P, P_proj;
		for (int j = j_start; j < skeleton->cols(); j++)
		{
			t = (skeleton->col(j) - A).dot(B - A) / (B - A).squaredNorm();
			P_proj = A + t * (B - A);
			r = std::max(r, (skeleton->col(j) - P_proj).norm() + capsules_radius[j-1]);
		}
	}
	return r;
}

fcl::Transform3f robots::xArm6::KDL2fcl(const KDL::Frame &in)
{
	fcl::Transform3f out;
    double x, y, z, w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vector3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternionf q(w, x, y, z);
    out.linear() = q.matrix();
    out.translation() = t;
    return out;
}

KDL::Frame robots::xArm6::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Matrix3f R = in.rotation();
	fcl::Quaternionf q(R);
    fcl::Vector3f t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
    return f;
}

void robots::xArm6::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box<float>(0.5, 1.0, 3.0));
	fcl::Transform3f tf; tf.translation() = fcl::Vector3f(1, 1, 1.5);
	std::unique_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject<float>(fclBox, tf));
	for (size_t i = 0; i < parts.size(); ++i)
	{
		fcl::DistanceRequest<float> request;
		fcl::DistanceResult<float> result;
		fcl::distance(parts[i].get(), ob.get(), request, result);
		LOG(INFO) << parts[i]->getAABB().min_ <<"\t;\t" << parts[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		LOG(INFO) << "distance from " << i+1 << ": " << result.min_distance << std::endl;
	}
}