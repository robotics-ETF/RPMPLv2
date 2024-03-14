#include <ostream>
#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

#include "Environment.h"

#include <glog/logging.h>

int main([[maybe_unused]] int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;

	env::Environment environment("/data/planar_2dof/obstacles_easy.yaml");

	return 0;
}
