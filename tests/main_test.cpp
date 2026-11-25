#include <gtest/gtest.h>
#include "tests_realvectorspacestate.h"
#include "tests_forward_kinematics.h"
#include "tests_distance.h"
#include "tests_onnx_models.h"

int main(int argc, char **argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}