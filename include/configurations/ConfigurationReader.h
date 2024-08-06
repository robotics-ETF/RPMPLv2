//
// Created by dinko on 17.02.22.
// Modified by nermin on 22.07.23.
//

#ifndef RPMPL_CONFIGURATIONREADER_H
#define RPMPL_CONFIGURATIONREADER_H

#include <string>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <glog/logging.h>

#include "RealVectorSpaceConfig.h"
#include "RRTConnectConfig.h"
#include "RBTConnectConfig.h"
#include "RGBTConnectConfig.h"
#include "RGBMTStarConfig.h"
#include "DRGBTConfig.h"
#include "SplinesConfig.h"

class ConfigurationReader
{
public:
    static void initConfiguration(const std::string &root_path = "");
};

#endif //RPMPL_CONFIGURATIONREADER_H