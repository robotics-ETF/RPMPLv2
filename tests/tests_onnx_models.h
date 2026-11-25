//
// Unit tests for the GAT-R ONNX models used by SL_DRGBT:
//   data/planar_2dof/onnx_models/GATR_4obs.onnx  (ETFSA: outputs/sweeps/planar_gat_reach/reach0/seed_46/GATR.onnx)
//   data/xarm6/onnx_models/GATR.onnx             (ETFSA: outputs/comparison/20261125_102550/xarm6/gat_reach/GATR.onnx)
// Reference predictions (tests/data/GATR_*_reference.bin) were computed for rows of each model's test split with the
// Python ONNX Runtime used in training (1.31.1). See tests/data/make_onnx_reference.py.
//

#include "ONNX/Predictor.h"
#include "SL_DRGBT.h"
#include "ConfigurationReader.h"
#include "Scenario.h"

#include <cmath>
#include <fstream>

struct OnnxModelCase
{
	std::string name;
	std::string model_path;			// Relative to the project
	std::string reference_path;		// Relative to the project
	std::string scenario_path;		// Relative to the project. Used to generate network inputs by SL_DRGBT
	size_t num_DOFs;
	size_t num_inputs;
	size_t num_outputs;
};

inline std::ostream &operator<<(std::ostream &os, const OnnxModelCase &model_case) { return os << model_case.name; }

namespace
{
	struct Reference
	{
		std::vector<std::vector<float>> inputs;
		std::vector<std::vector<float>> outputs;
	};

	// Format: uint32 num_rows, num_inputs, num_outputs, then per row float32 inputs followed by float32 outputs.
	Reference readReference(const std::string &path, size_t num_inputs, size_t num_outputs)
	{
		std::ifstream file(path, std::ios::binary);
		uint32_t header[3] {};
		if (!file.read(reinterpret_cast<char *>(header), sizeof(header)) || header[1] != num_inputs || header[2] != num_outputs)
			throw std::runtime_error("Invalid reference file: " + path);

		Reference reference {};
		for (uint32_t k = 0; k < header[0]; k++)
		{
			std::vector<float> input(num_inputs), output(num_outputs);
			if (!file.read(reinterpret_cast<char *>(input.data()), input.size() * sizeof(float)) ||
				!file.read(reinterpret_cast<char *>(output.data()), output.size() * sizeof(float)))
				throw std::runtime_error("Truncated reference file: " + path);

			reference.inputs.emplace_back(input);
			reference.outputs.emplace_back(output);
		}
		return reference;
	}

	// Root distribution over 2n directions, followed by a conditional distribution over the remaining 2n-1 directions
	// for each of them, e.g., [0,4), [4,7), [7,10), [10,13), [13,16) for n = 2 (equal to 'group_bounds' in model metadata).
	std::vector<std::pair<size_t, size_t>> outputGroups(size_t num_DOFs)
	{
		std::vector<std::pair<size_t, size_t>> groups { { 0, 2 * num_DOFs } };
		for (size_t k = 0; k < 2 * num_DOFs; k++)
			groups.emplace_back(groups.back().second, groups.back().second + 2 * num_DOFs - 1);

		return groups;
	}

	// Tolerances are 'group_sum_tolerance' and 'probability_bound_tolerance' from the model metadata.
	void expectValidProbabilities(const Predictor::Probabilities &output, size_t num_DOFs)
	{
		std::vector<std::pair<size_t, size_t>> groups { outputGroups(num_DOFs) };
		ASSERT_EQ(groups.back().second, output.size());

		for (const auto &[first, last] : groups)
		{
			float sum { 0 };
			for (size_t i = first; i < last; i++)
			{
				ASSERT_TRUE(std::isfinite(output[i]));
				EXPECT_GE(output[i], -1e-6);
				EXPECT_LE(output[i], 1 + 1e-6);
				sum += output[i];
			}
			EXPECT_NEAR(sum, 1, 1e-3) << "Group [" << first << ", " << last << ")";
		}
	}
}

class OnnxModelTest : public ::testing::TestWithParam<OnnxModelCase>
{
protected:
	std::string path(const std::string &relative_path) const { return std::string(RPMPL_PROJECT_PATH) + relative_path; }
};

TEST_P(OnnxModelTest, interface)
{
	Predictor predictor(path(GetParam().model_path));
	EXPECT_EQ(predictor.getInputCount(), GetParam().num_inputs);
	EXPECT_EQ(predictor.getOutputCount(), GetParam().num_outputs);
}

// RPMPLv2 (ONNX Runtime 1.23.2) must give the same predictions as the training environment.
TEST_P(OnnxModelTest, matchesTrainingRuntime)
{
	const float tolerance { 1e-5 };
	Predictor predictor(path(GetParam().model_path));
	Reference reference { readReference(path(GetParam().reference_path), GetParam().num_inputs, GetParam().num_outputs) };
	ASSERT_FALSE(reference.inputs.empty());

	for (size_t k = 0; k < reference.inputs.size(); k++)
	{
		const Predictor::Probabilities output { predictor.predict(reference.inputs[k]) };
		for (size_t i = 0; i < output.size(); i++)
			ASSERT_NEAR(output[i], reference.outputs[k][i], tolerance) << "Row " << k << ", output " << i;

		for (const auto &[first, last] : outputGroups(GetParam().num_DOFs))	// The same move is selected
		{
			EXPECT_EQ(std::max_element(output.begin() + first, output.begin() + last) - output.begin(),
					  std::max_element(reference.outputs[k].begin() + first, reference.outputs[k].begin() + last) - reference.outputs[k].begin())
				<< "Row " << k << ", group [" << first << ", " << last << ")";
		}

		expectValidProbabilities(output, GetParam().num_DOFs);
	}
}

TEST_P(OnnxModelTest, deterministic)
{
	Predictor predictor(path(GetParam().model_path));
	Reference reference { readReference(path(GetParam().reference_path), GetParam().num_inputs, GetParam().num_outputs) };

	const Predictor::Probabilities output { predictor.predict(reference.inputs.front()) };
	predictor.predict(reference.inputs.back());
	EXPECT_EQ(predictor.predict(reference.inputs.front()), output);
}

TEST_P(OnnxModelTest, rejectsInvalidInput)
{
	Predictor predictor(path(GetParam().model_path));
	EXPECT_THROW(predictor.predict(std::vector<float>(GetParam().num_inputs - 1, 0)), std::invalid_argument);
	EXPECT_THROW(predictor.predict(std::vector<float>(GetParam().num_inputs + 1, 0)), std::invalid_argument);

	std::vector<float> input(GetParam().num_inputs, 0);
	input[GetParam().num_inputs / 2] = NAN;
	EXPECT_THROW(predictor.predict(input), std::invalid_argument);
}

// Inputs generated by SL_DRGBT have the size expected by the model and give valid probabilities.
TEST_P(OnnxModelTest, inputsFromPlanner)
{
	ConfigurationReader::initConfiguration(RPMPL_PROJECT_PATH);
	scenario::Scenario scenario(GetParam().scenario_path, RPMPL_PROJECT_PATH);
	ASSERT_EQ(scenario.getStateSpace()->num_dimensions, GetParam().num_DOFs);

	SL_DRGBTConfig::ONNX_MODEL_PATH = path(GetParam().model_path);
	planning::drbt::SL_DRGBT planner(scenario.getStateSpace(), scenario.getStart(), scenario.getGoal());
	Predictor predictor(SL_DRGBTConfig::ONNX_MODEL_PATH);

	std::vector<float> input {};
	planner.generateInput(scenario.getStart(), input);
	ASSERT_EQ(input.size(), GetParam().num_inputs);
	expectValidProbabilities(predictor.predict(input), GetParam().num_DOFs);
}

INSTANTIATE_TEST_SUITE_P(GATR, OnnxModelTest, ::testing::Values(
	OnnxModelCase { "planar_2dof", "/data/planar_2dof/onnx_models/GATR_4obs.onnx", "/tests/data/GATR_planar_2dof_reference.bin",
					"/data/planar_2dof/scenario1/scenario1.yaml", 2, 192, 16 },
	OnnxModelCase { "xarm6", "/data/xarm6/onnx_models/GATR.onnx", "/tests/data/GATR_xarm6_reference.bin",
					"/data/xarm6/scenario_random_obstacles/scenario_random_obstacles1.yaml", 6, 5088, 144 }
), [](const ::testing::TestParamInfo<OnnxModelCase> &info) { return info.param.name; });
