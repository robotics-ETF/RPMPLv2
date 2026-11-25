//
// Regression test: robot FK must give exactly the same frames as the original per-segment KDL tree solver.
// The ONNX models are trained on features derived from these frames, so any change in FK would invalidate them.
//

#include "Planar2DOF.h"
#include "Planar10DOF.h"
#include "Spatial10DOF.h"
#include "Spatial22DOF.h"
#include "xArm6.h"

#include <random>

namespace
{
	// Original implementation of 'computeForwardKinematics': one tree solver call per segment of the chain.
	std::vector<KDL::Frame> referenceForwardKinematics(const KDL::Tree &tree, const std::string &root, const std::string &tip,
													   const Eigen::VectorXf &q, size_t num_frames, float gripper_length = 0)
	{
		KDL::Chain chain {};
		tree.getChain(root, tip, chain);
		KDL::TreeFkSolverPos_recursive tree_fk_solver(tree);
		KDL::JntArray joint_pos(q.size());
		for (int i = 0; i < q.size(); i++)
			joint_pos(i) = q(i);

		std::vector<KDL::Frame> frames(num_frames);
		for (size_t i = 0; i < num_frames; i++)
			tree_fk_solver.JntToCart(joint_pos, frames[i], chain.getSegment(i).getName());
		frames.back().p += gripper_length * frames.back().M.UnitZ();

		return frames;
	}

	// Bitwise equality ('KDL::Equal' uses strict inequalities, so it cannot be used with zero tolerance).
	bool isSameFrame(const KDL::Frame &frame1, const KDL::Frame &frame2)
	{
		for (size_t i = 0; i < 3; i++)
		{
			if (frame1.p(i) != frame2.p(i))
				return false;
			for (size_t j = 0; j < 3; j++)
				if (frame1.M(i, j) != frame2.M(i, j))
					return false;
		}
		return true;
	}

	void expectSameFrames(robots::AbstractRobot &robot, const KDL::Tree &tree, const std::string &root, const std::string &tip,
						  size_t num_frames, float gripper_length = 0)
	{
		std::mt19937 generator(1);
		for (size_t n = 0; n < 1000; n++)
		{
			Eigen::VectorXf q(robot.getNumDOFs());
			for (size_t i = 0; i < robot.getNumDOFs(); i++)
				q(i) = std::uniform_real_distribution<float>(robot.getLimits()[i].first, robot.getLimits()[i].second)(generator);

			std::vector<KDL::Frame> expected { referenceForwardKinematics(tree, root, tip, q, num_frames, gripper_length) };
			std::shared_ptr<std::vector<KDL::Frame>> frames
				{ robot.computeForwardKinematics(std::make_shared<base::RealVectorSpaceState>(q)) };

			ASSERT_EQ(frames->size(), expected.size());
			for (size_t i = 0; i < expected.size(); i++)
			{
				// Exact equality: the chain solver performs the same floating-point operations as the tree solver
				ASSERT_TRUE(isSameFrame(frames->at(i), expected[i])) 
					<< "Frame " << i << " differs for q = " << q.transpose();
			}
		}
	}

	std::string urdfPath(const std::string &robot_name)
	{
		return std::string(RPMPL_PROJECT_PATH) + "/data/" + robot_name + "/" + robot_name + ".urdf";
	}
}

TEST(ForwardKinematicsTest, planar2DOF)
{
	robots::Planar2DOF robot(urdfPath("planar_2dof"));
	expectSameFrames(robot, robot.getRobotTree(), "base_link", "tool", robot.getRobotTree().getNrOfSegments());
}

TEST(ForwardKinematicsTest, planar10DOF)
{
	robots::Planar10DOF robot(urdfPath("planar_10dof"));
	expectSameFrames(robot, robot.getRobotTree(), "base_link", "tool", robot.getRobotTree().getNrOfSegments());
}

TEST(ForwardKinematicsTest, spatial10DOF)
{
	robots::Spatial10DOF robot(urdfPath("spatial_10dof"), 0);
	expectSameFrames(robot, robot.getRobotTree(), "base_link", "tool", robot.getRobotTree().getNrOfSegments());
}

TEST(ForwardKinematicsTest, spatial22DOF)
{
	robots::Spatial22DOF robot(urdfPath("spatial_22dof"), 0);
	expectSameFrames(robot, robot.getRobotTree(), "base_link", "tool", robot.getRobotTree().getNrOfSegments());
}

TEST(ForwardKinematicsTest, xArm6)
{
	robots::xArm6 robot(urdfPath("xarm6"));
	expectSameFrames(robot, robot.getRobotTree(), "link_base", "link_eef", robot.getNumDOFs());
}

TEST(ForwardKinematicsTest, xArm6WithGripper)
{
	robots::xArm6 robot(urdfPath("xarm6"), 0.1);
	expectSameFrames(robot, robot.getRobotTree(), "link_base", "link_eef", robot.getNumDOFs(), 0.1);
}
