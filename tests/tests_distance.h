//
// Regression test: distance and collision queries must give bitwise identical results to the original implementation.
// Expected hashes were recorded with the original code (commit 04e5ec4). Features of the trained ONNX models are built
// from these results, so any change here would invalidate the models.
//

#include "CollisionAndDistance.h"
#include "ConfigurationReader.h"
#include "Scenario.h"

#include <cstring>
#include <random>

namespace
{
	// Uniform floats from mt19937, whose output is fixed by the C++ standard (unlike std::uniform_real_distribution).
	class DeterministicRandom
	{
	public:
		explicit DeterministicRandom(uint32_t seed) : generator(seed) {}
		float uniform(float low, float high) { return low + (high - low) * float(generator() >> 8) * (1.0f / 16777216.0f); }
		Eigen::Vector3f uniform3(float low, float high)
		{
			// Separate statements, since the evaluation order of function arguments differs between compilers
			float x { uniform(low, high) };
			float y { uniform(low, high) };
			float z { uniform(low, high) };
			return Eigen::Vector3f(x, y, z);
		}

	private:
		std::mt19937 generator;
	};

	// FNV-1a hash of exact float bits.
	class BitHash
	{
	public:
		void add(float value)
		{
			uint32_t bits {};
			std::memcpy(&bits, &value, sizeof(bits));
			for (size_t k = 0; k < 4; k++)
				hash = (hash ^ ((bits >> (8 * k)) & 0xFF)) * 1099511628211ull;
		}
		template <class Matrix>
		void addMatrix(const Matrix &matrix)
		{
			for (int j = 0; j < matrix.cols(); j++)
				for (int i = 0; i < matrix.rows(); i++)
					add(matrix(i, j));
		}
		uint64_t get() const { return hash; }

	private:
		uint64_t hash { 14695981039346656037ull };
	};

	// Nearest points are valid only for a positive distance.
	void addDistance(BitHash &hash, const std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> &result)
	{
		hash.add(std::get<0>(result));
		if (std::get<0>(result) > 0)
			hash.addMatrix(*std::get<1>(result));
	}
}

TEST(DistanceRegressionTest, primitives)
{
	DeterministicRandom random(2024);
	BitHash hash_capsule_box {}, hash_line_seg {}, hash_point {}, hash_sphere {};

	for (size_t n = 0; n < 20000; n++)
	{
		Eigen::Vector3f box_min { random.uniform3(-0.5, 0.5) };
		Eigen::Vector3f box_dim { random.uniform3(0.05, 0.6) };
		Eigen::Vector3f A { random.uniform3(-1, 1) };
		Eigen::Vector3f B { random.uniform3(-1, 1) };
		Eigen::Vector3f C { random.uniform3(-1, 1) };
		Eigen::Vector3f D { random.uniform3(-1, 1) };
		float radius { random.uniform(0, 0.1) };

		switch (n % 4)	// Besides general positions, cover planar, coplanar and parallel (degenerate) cases
		{
		case 1:	// Horizontal segments
			B(2) = A(2);
			D(2) = C(2);
			break;
		case 2:	// Planar robot: segments in z = 0, and box crossing that plane
			A(2) = B(2) = C(2) = D(2) = 0;
			box_min(2) = -box_dim(2) / 2;
			break;
		case 3:	// Segments parallel to each other and to a box edge
			B = A + Eigen::Vector3f(0, 0, box_dim(0));
			D = C + Eigen::Vector3f(0, 0, box_dim(1));
			break;
		}

		Eigen::VectorXf box(6);
		box << box_min, box_min + box_dim;
		Eigen::VectorXf sphere(4);
		sphere << C, radius + 0.05f;

		addDistance(hash_capsule_box, base::CollisionAndDistance::distanceCapsuleToBox(A, B, radius, box));
		hash_capsule_box.add(base::CollisionAndDistance::collisionCapsuleToBox(A, B, radius, box));
		addDistance(hash_line_seg, base::CollisionAndDistance::distanceLineSegToLineSeg(A, B, C, D));
		addDistance(hash_point, base::CollisionAndDistance::distanceLineSegToPoint(A, B, C));
		addDistance(hash_sphere, base::CollisionAndDistance::distanceCapsuleToSphere(A, B, radius, sphere));
		hash_sphere.add(base::CollisionAndDistance::collisionCapsuleToSphere(A, B, radius, sphere));
	}

	EXPECT_EQ(hash_capsule_box.get(), 0x1bbf5169bc33a4beull);
	EXPECT_EQ(hash_line_seg.get(), 0x28818dabfb9ec32aull);
	EXPECT_EQ(hash_point.get(), 0x9e3abe0897aef352ull);
	EXPECT_EQ(hash_sphere.get(), 0x5f99fe76dd26bf2bull);
}

class DistanceScenarioTest : public ::testing::TestWithParam<std::tuple<std::string, uint64_t>> {};

TEST_P(DistanceScenarioTest, distanceProfiles)
{
	const std::string project_path { RPMPL_PROJECT_PATH };
	ConfigurationReader::initConfiguration(project_path);
	scenario::Scenario scenario(std::get<0>(GetParam()), project_path);
	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	DeterministicRandom random(7);
	BitHash hash {};

	for (size_t n = 0; n < 1000; n++)
	{
		Eigen::VectorXf coord(ss->num_dimensions), delta(ss->num_dimensions);
		for (size_t i = 0; i < ss->num_dimensions; i++)
		{
			coord(i) = random.uniform(ss->robot->getLimits()[i].first, ss->robot->getLimits()[i].second);
			delta(i) = random.uniform(-0.05, 0.05);
		}

		std::shared_ptr<base::State> q { ss->getNewState(coord) };
		hash.add(ss->computeDistance(q, true));
		for (float d_c : q->getDistanceProfile())
			hash.add(d_c);

		if (q->getNearestPoints() != nullptr)
		{
			for (const Eigen::MatrixXf &nearest_pts : *q->getNearestPoints())
				hash.addMatrix(nearest_pts);

			std::shared_ptr<base::State> q_near { ss->getNewState(coord + delta) };
			hash.add(ss->computeDistanceUnderestimation(q_near, q->getNearestPoints()));
			for (float d_c : q_near->getDistanceProfile())
				hash.add(d_c);
		}

		hash.add(ss->isValid(q));
		// 'checkSelfCollision' is not hashed: for xArm6 with a gripper, its result depends on uninitialised memory
		// (the gripper's FCL AABB is never computed from its actual pose; see TODO in 'xArm6::checkRealSelfCollision').
	}

	EXPECT_EQ(hash.get(), std::get<1>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(Scenarios, DistanceScenarioTest, ::testing::Values(
	std::make_tuple("/data/xarm6/scenario1/scenario1.yaml", 0xd25aeeec9b7fa9efull),
	std::make_tuple("/data/spatial_10dof/scenario1/scenario1.yaml", 0xbd8c4310f3870a6cull)
));
