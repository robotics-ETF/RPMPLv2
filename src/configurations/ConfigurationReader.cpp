#include "ConfigurationReader.h"

void ConfigurationReader::initConfiguration(const std::string &root_path)
{
    YAML::Node RealVectorSpaceConfigRoot    { YAML::LoadFile(root_path + "/data/configurations/configuration_realvectorspace.yaml") };
    YAML::Node RRTConnectConfigRoot         { YAML::LoadFile(root_path + "/data/configurations/configuration_rrtconnect.yaml") };
    YAML::Node RBTConnectConfigRoot         { YAML::LoadFile(root_path + "/data/configurations/configuration_rbtconnect.yaml") };
    YAML::Node RGBTConnectConfigRoot        { YAML::LoadFile(root_path + "/data/configurations/configuration_rgbtconnect.yaml") };
    YAML::Node RGBMTStarConfigRoot          { YAML::LoadFile(root_path + "/data/configurations/configuration_rgbmtstar.yaml") };
    YAML::Node DRGBTConfigRoot              { YAML::LoadFile(root_path + "/data/configurations/configuration_drgbt.yaml") };
    YAML::Node SplinesConfigRoot            { YAML::LoadFile(root_path + "/data/configurations/configuration_splines.yaml") };

    // RealVectorSpaceConfigRoot
    if (RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].IsDefined())
        RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS = RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].as<size_t>();
    else
        LOG(INFO) << "RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS is not defined! Using default value of " << RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
    
    if (RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].IsDefined())
        RealVectorSpaceConfig::EQUALITY_THRESHOLD = RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].as<float>();
    else
        LOG(INFO) << "RealVectorSpaceConfig::EQUALITY_THRESHOLD is not defined! Using default value of " << RealVectorSpaceConfig::EQUALITY_THRESHOLD;

    // RRTConnectConfigRoot
    if (RRTConnectConfigRoot["MAX_NUM_ITER"].IsDefined())
        RRTConnectConfig::MAX_NUM_ITER = RRTConnectConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RRTConnectConfig::MAX_NUM_ITER is not defined! Using default value of " << RRTConnectConfig::MAX_NUM_ITER;
    
    if (RRTConnectConfigRoot["MAX_NUM_STATES"].IsDefined())
        RRTConnectConfig::MAX_NUM_STATES = RRTConnectConfigRoot["MAX_NUM_STATES"].as<size_t>();
    else
        LOG(INFO) << "RRTConnectConfig::MAX_NUM_STATES is not defined! Using default value of " << RRTConnectConfig::MAX_NUM_STATES;
    
    if (RRTConnectConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RRTConnectConfig::MAX_PLANNING_TIME = RRTConnectConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RRTConnectConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RRTConnectConfig::MAX_PLANNING_TIME;
    
    if (RRTConnectConfigRoot["MAX_EXTENSION_STEPS"].IsDefined())
        RRTConnectConfig::MAX_EXTENSION_STEPS = RRTConnectConfigRoot["MAX_EXTENSION_STEPS"].as<size_t>();
    else
        LOG(INFO) << "RRTConnectConfig::MAX_EXTENSION_STEPS is not defined! Using default value of " << RRTConnectConfig::MAX_EXTENSION_STEPS;
    
    if (RRTConnectConfigRoot["EPS_STEP"].IsDefined())
        RRTConnectConfig::EPS_STEP = RRTConnectConfigRoot["EPS_STEP"].as<float>();
    else
        LOG(INFO) << "RRTConnectConfig::EPS_STEP is not defined! Using default value of " << RRTConnectConfig::EPS_STEP;

    // RBTConnectConfigRoot
    if (RBTConnectConfigRoot["MAX_NUM_ITER"].IsDefined())
        RBTConnectConfig::MAX_NUM_ITER = RBTConnectConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RBTConnectConfig::MAX_NUM_ITER is not defined! Using default value of " << RBTConnectConfig::MAX_NUM_ITER;
    
    if (RBTConnectConfigRoot["MAX_NUM_STATES"].IsDefined())
        RBTConnectConfig::MAX_NUM_STATES = RBTConnectConfigRoot["MAX_NUM_STATES"].as<size_t>();
    else
        LOG(INFO) << "RBTConnectConfig::MAX_NUM_STATES is not defined! Using default value of " << RBTConnectConfig::MAX_NUM_STATES;
    
    if (RBTConnectConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RBTConnectConfig::MAX_PLANNING_TIME = RBTConnectConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RBTConnectConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RBTConnectConfig::MAX_PLANNING_TIME;
    
    if (RBTConnectConfigRoot["D_CRIT"].IsDefined())
        RBTConnectConfig::D_CRIT = RBTConnectConfigRoot["D_CRIT"].as<float>();
    else
        LOG(INFO) << "RBTConnectConfig::D_CRIT is not defined! Using default value of " << RBTConnectConfig::D_CRIT;
    
    if (RBTConnectConfigRoot["DELTA"].IsDefined())
        RBTConnectConfig::DELTA = RBTConnectConfigRoot["DELTA"].as<float>();
    else
        LOG(INFO) << "RBTConnectConfig::DELTA is not defined! Using default value of " << RBTConnectConfig::DELTA;
    
    if (RBTConnectConfigRoot["NUM_SPINES"].IsDefined())
        RBTConnectConfig::NUM_SPINES = RBTConnectConfigRoot["NUM_SPINES"].as<size_t>();
    else
        LOG(INFO) << "RBTConnectConfig::NUM_SPINES is not defined! Using default value of " << RBTConnectConfig::NUM_SPINES;
    
    if (RBTConnectConfigRoot["NUM_ITER_SPINE"].IsDefined())
        RBTConnectConfig::NUM_ITER_SPINE = RBTConnectConfigRoot["NUM_ITER_SPINE"].as<size_t>();
    else
        LOG(INFO) << "RBTConnectConfig::NUM_ITER_SPINE is not defined! Using default value of " << RBTConnectConfig::NUM_ITER_SPINE;

    if (RBTConnectConfigRoot["USE_EXPANDED_BUBBLE"].IsDefined())
        RBTConnectConfig::USE_EXPANDED_BUBBLE = RBTConnectConfigRoot["USE_EXPANDED_BUBBLE"].as<bool>();
    else
        LOG(INFO) << "RBTConnectConfig::USE_EXPANDED_BUBBLE is not defined! Using default value of " << RBTConnectConfig::USE_EXPANDED_BUBBLE;

    // RGBTConnectConfigRoot
    if (RGBTConnectConfigRoot["MAX_NUM_ITER"].IsDefined())
        RGBTConnectConfig::MAX_NUM_ITER = RGBTConnectConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RGBTConnectConfig::MAX_NUM_ITER is not defined! Using default value of " << RGBTConnectConfig::MAX_NUM_ITER;
    
    if (RGBTConnectConfigRoot["MAX_NUM_STATES"].IsDefined())
        RGBTConnectConfig::MAX_NUM_STATES = RGBTConnectConfigRoot["MAX_NUM_STATES"].as<size_t>();
    else
        LOG(INFO) << "RGBTConnectConfig::MAX_NUM_STATES is not defined! Using default value of " << RGBTConnectConfig::MAX_NUM_STATES;
    
    if (RGBTConnectConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RGBTConnectConfig::MAX_PLANNING_TIME = RGBTConnectConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RGBTConnectConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RGBTConnectConfig::MAX_PLANNING_TIME;
    
    if (RGBTConnectConfigRoot["NUM_LAYERS"].IsDefined())
        RGBTConnectConfig::NUM_LAYERS = RGBTConnectConfigRoot["NUM_LAYERS"].as<size_t>();
    else
        LOG(INFO) << "RGBTConnectConfig::NUM_LAYERS is not defined! Using default value of " << RGBTConnectConfig::NUM_LAYERS;
    
    // RGBMTStarConfigRoot
    if (RGBMTStarConfigRoot["MAX_NUM_ITER"].IsDefined())
        RGBMTStarConfig::MAX_NUM_ITER = RGBMTStarConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RGBMTStarConfig::MAX_NUM_ITER is not defined! Using default value of " << RGBMTStarConfig::MAX_NUM_ITER;
    
    if (RGBMTStarConfigRoot["MAX_NUM_STATES"].IsDefined())
        RGBMTStarConfig::MAX_NUM_STATES = RGBMTStarConfigRoot["MAX_NUM_STATES"].as<size_t>();
    else
        LOG(INFO) << "RGBMTStarConfig::MAX_NUM_STATES is not defined! Using default value of " << RGBMTStarConfig::MAX_NUM_STATES;
    
    if (RGBMTStarConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RGBMTStarConfig::MAX_PLANNING_TIME = RGBMTStarConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RGBMTStarConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RGBMTStarConfig::MAX_PLANNING_TIME;
    
    if (RGBMTStarConfigRoot["TERMINATE_WHEN_PATH_IS_FOUND"].IsDefined())
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = RGBMTStarConfigRoot["TERMINATE_WHEN_PATH_IS_FOUND"].as<bool>();
    else
        LOG(INFO) << "RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND is not defined! Using default value of " << RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND;

    // DRGBTConfigRoot
    if (DRGBTConfigRoot["MAX_NUM_ITER"].IsDefined())
        DRGBTConfig::MAX_NUM_ITER = DRGBTConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "DRGBTConfig::MAX_NUM_ITER is not defined! Using default value of " << DRGBTConfig::MAX_NUM_ITER;
    
    if (DRGBTConfigRoot["MAX_ITER_TIME"].IsDefined())
        DRGBTConfig::MAX_ITER_TIME = DRGBTConfigRoot["MAX_ITER_TIME"].as<float>();
    else
        LOG(INFO) << "DRGBTConfig::MAX_ITER_TIME is not defined! Using default value of " << DRGBTConfig::MAX_ITER_TIME;
    
    if (DRGBTConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        DRGBTConfig::MAX_PLANNING_TIME = DRGBTConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "DRGBTConfig::MAX_PLANNING_TIME is not defined! Using default value of " << DRGBTConfig::MAX_PLANNING_TIME;
    
    if (DRGBTConfigRoot["INIT_HORIZON_SIZE"].IsDefined())
        DRGBTConfig::INIT_HORIZON_SIZE = DRGBTConfigRoot["INIT_HORIZON_SIZE"].as<size_t>();
    else
        LOG(INFO) << "DRGBTConfig::INIT_HORIZON_SIZE is not defined! Using default value of " << DRGBTConfig::INIT_HORIZON_SIZE;
    
    if (DRGBTConfigRoot["TRESHOLD_WEIGHT"].IsDefined())
        DRGBTConfig::TRESHOLD_WEIGHT = DRGBTConfigRoot["TRESHOLD_WEIGHT"].as<float>();
    else
        LOG(INFO) << "DRGBTConfig::TRESHOLD_WEIGHT is not defined! Using default value of " << DRGBTConfig::TRESHOLD_WEIGHT;
    
    if (DRGBTConfigRoot["D_CRIT"].IsDefined())
        DRGBTConfig::D_CRIT = DRGBTConfigRoot["D_CRIT"].as<float>();
    else
        LOG(INFO) << "DRGBTConfig::D_CRIT is not defined! Using default value of " << DRGBTConfig::D_CRIT;

    if (DRGBTConfigRoot["MAX_NUM_MODIFY_ATTEMPTS"].IsDefined())
        DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS = DRGBTConfigRoot["MAX_NUM_MODIFY_ATTEMPTS"].as<size_t>();
    else
        LOG(INFO) << "DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS is not defined! Using default value of " << DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS;

    if (DRGBTConfigRoot["STATIC_PLANNER_TYPE"].IsDefined())
        DRGBTConfig::STATIC_PLANNER_TYPE = planning::planner_type_map[DRGBTConfigRoot["STATIC_PLANNER_TYPE"].as<std::string>()];
    else
        LOG(INFO) << "DRGBTConfig::STATIC_PLANNER_TYPE is not defined! Using default value of " << DRGBTConfig::STATIC_PLANNER_TYPE;

    if (DRGBTConfigRoot["REAL_TIME_SCHEDULING"].IsDefined())
        DRGBTConfig::REAL_TIME_SCHEDULING = planning::real_time_scheduling_map[DRGBTConfigRoot["REAL_TIME_SCHEDULING"].as<std::string>()];
    else
        LOG(INFO) << "DRGBTConfig::REAL_TIME_SCHEDULING is not defined! Using default value of " << DRGBTConfig::REAL_TIME_SCHEDULING;
    
    if (DRGBTConfigRoot["MAX_TIME_TASK1"].IsDefined())
        DRGBTConfig::MAX_TIME_TASK1 = DRGBTConfigRoot["MAX_TIME_TASK1"].as<float>();
    else
        LOG(INFO) << "DRGBTConfig::MAX_TIME_TASK1 is not defined! Using default value of " << DRGBTConfig::MAX_TIME_TASK1;
    
    if (DRGBTConfigRoot["TRAJECTORY_INTERPOLATION"].IsDefined())
        DRGBTConfig::TRAJECTORY_INTERPOLATION = planning::trajectory_interpolation_map[DRGBTConfigRoot["TRAJECTORY_INTERPOLATION"].as<std::string>()];
    else
        LOG(INFO) << "DRGBTConfig::TRAJECTORY_INTERPOLATION is not defined! Using default value of " << DRGBTConfig::TRAJECTORY_INTERPOLATION;
    
    if (DRGBTConfigRoot["GUARANTEED_SAFE_MOTION"].IsDefined())
        DRGBTConfig::GUARANTEED_SAFE_MOTION = DRGBTConfigRoot["GUARANTEED_SAFE_MOTION"].as<bool>();
    else
        LOG(INFO) << "DRGBTConfig::GUARANTEED_SAFE_MOTION is not defined! Using default value of " << DRGBTConfig::GUARANTEED_SAFE_MOTION;

    // SplinesConfigRoot
    if (SplinesConfigRoot["MAX_TIME_COMPUTE_REGULAR"].IsDefined())
        SplinesConfig::MAX_TIME_COMPUTE_REGULAR = SplinesConfigRoot["MAX_TIME_COMPUTE_REGULAR"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::MAX_TIME_COMPUTE_REGULAR is not defined! Using default value of " << SplinesConfig::MAX_TIME_COMPUTE_REGULAR;

    if (SplinesConfigRoot["MAX_TIME_COMPUTE_SAFE"].IsDefined())
        SplinesConfig::MAX_TIME_COMPUTE_SAFE = SplinesConfigRoot["MAX_TIME_COMPUTE_SAFE"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::MAX_TIME_COMPUTE_SAFE is not defined! Using default value of " << SplinesConfig::MAX_TIME_COMPUTE_SAFE;

    if (SplinesConfigRoot["MAX_TIME_PUBLISH"].IsDefined())
        SplinesConfig::MAX_TIME_PUBLISH = SplinesConfigRoot["MAX_TIME_PUBLISH"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::MAX_TIME_PUBLISH is not defined! Using default value of " << SplinesConfig::MAX_TIME_PUBLISH;

    if (SplinesConfigRoot["MAX_TIME_FINAL"].IsDefined())
        SplinesConfig::MAX_TIME_FINAL = SplinesConfigRoot["MAX_TIME_FINAL"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::MAX_TIME_FINAL is not defined! Using default value of " << SplinesConfig::MAX_TIME_FINAL;

    if (SplinesConfigRoot["TIME_STEP"].IsDefined())
        SplinesConfig::TIME_STEP = SplinesConfigRoot["TIME_STEP"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::TIME_STEP is not defined! Using default value of " << SplinesConfig::TIME_STEP;

    if (SplinesConfigRoot["FINAL_JERK_STEP"].IsDefined())
        SplinesConfig::FINAL_JERK_STEP = SplinesConfigRoot["FINAL_JERK_STEP"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::FINAL_JERK_STEP is not defined! Using default value of " << SplinesConfig::FINAL_JERK_STEP;
    
    if (SplinesConfigRoot["FINAL_VELOCITY_STEP"].IsDefined())
        SplinesConfig::FINAL_VELOCITY_STEP = SplinesConfigRoot["FINAL_VELOCITY_STEP"].as<float>();
    else
        LOG(INFO) << "SplinesConfig::FINAL_VELOCITY_STEP is not defined! Using default value of " << SplinesConfig::FINAL_VELOCITY_STEP;
    
    LOG(INFO) << "Configuration parameters read successfully!";
}
