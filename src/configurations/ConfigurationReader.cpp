#include "ConfigurationReader.h"

void ConfigurationReader::initConfiguration(const std::string &root_path)
{
    YAML::Node RealVectorSpaceConfigRoot    { YAML::LoadFile(root_path + "/data/configurations/configuration_realvectorspace.yaml") };
    YAML::Node RRTConnectConfigRoot         { YAML::LoadFile(root_path + "/data/configurations/configuration_rrtconnect.yaml") };
    YAML::Node RBTConnectConfigRoot         { YAML::LoadFile(root_path + "/data/configurations/configuration_rbtconnect.yaml") };
    YAML::Node RGBTConnectConfigRoot        { YAML::LoadFile(root_path + "/data/configurations/configuration_rgbtconnect.yaml") };
    YAML::Node RGBMTStarConfigRoot          { YAML::LoadFile(root_path + "/data/configurations/configuration_rgbmtstar.yaml") };
    YAML::Node DRGBTConfigRoot              { YAML::LoadFile(root_path + "/data/configurations/configuration_drgbt.yaml") };
    YAML::Node RRTxConfigRoot               { YAML::LoadFile(root_path + "/data/configurations/configuration_rrtx.yaml") };
    YAML::Node TrajectoryConfigRoot         { YAML::LoadFile(root_path + "/data/configurations/configuration_trajectory.yaml") };
    YAML::Node RT_RGBTConfigRoot            { YAML::LoadFile(root_path + "/data/configurations/configuration_rt_rgbt.yaml") };

    // RealVectorSpaceConfigRoot
    if (RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].IsDefined())
        RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS = RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].as<size_t>();
    else
        LOG(INFO) << "RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS is not defined! Using default value of " << RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
    
    if (RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].IsDefined())
        RealVectorSpaceConfig::EQUALITY_THRESHOLD = RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].as<float>();
    else
        LOG(INFO) << "RealVectorSpaceConfig::EQUALITY_THRESHOLD is not defined! Using default value of " << RealVectorSpaceConfig::EQUALITY_THRESHOLD;
    
    if (RealVectorSpaceConfigRoot["MAX_DISTANCE"].IsDefined())
        RealVectorSpaceConfig::MAX_DISTANCE = RealVectorSpaceConfigRoot["MAX_DISTANCE"].as<float>();
    else
        LOG(INFO) << "RealVectorSpaceConfig::MAX_DISTANCE is not defined! Using default value of " << RealVectorSpaceConfig::MAX_DISTANCE;

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

    if (RGBMTStarConfigRoot["SAFETY_FACTOR"].IsDefined())
        RGBMTStarConfig::SAFETY_FACTOR = RGBMTStarConfigRoot["SAFETY_FACTOR"].as<float>();
    else
        LOG(INFO) << "RGBMTStarConfig::SAFETY_FACTOR is not defined! Using default value of " << RGBMTStarConfig::SAFETY_FACTOR;
    
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

    // RRTxConfigRoot
    if (RRTxConfigRoot["MAX_NUM_ITER"].IsDefined())
        RRTxConfig::MAX_NUM_ITER = RRTxConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RRTxConfig::MAX_NUM_ITER is not defined! Using default value of " << RRTxConfig::MAX_NUM_ITER;

    if (RRTxConfigRoot["MAX_ITER_TIME"].IsDefined())
        RRTxConfig::MAX_ITER_TIME = RRTxConfigRoot["MAX_ITER_TIME"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::MAX_ITER_TIME is not defined! Using default value of " << RRTxConfig::MAX_ITER_TIME;

    if (RRTxConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RRTxConfig::MAX_PLANNING_TIME = RRTxConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RRTxConfig::MAX_PLANNING_TIME;

    if (RRTxConfigRoot["EPS_STEP"].IsDefined())
        RRTxConfig::EPS_STEP = RRTxConfigRoot["EPS_STEP"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::EPS_STEP is not defined! Using default value of " << RRTxConfig::EPS_STEP;

    if (RRTxConfigRoot["R_REWIRE"].IsDefined())
        RRTxConfig::R_REWIRE = RRTxConfigRoot["R_REWIRE"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::R_REWIRE is not defined! Using default value of " << RRTxConfig::R_REWIRE;

    if (RRTxConfigRoot["MAX_NEIGHBORS"].IsDefined())
        RRTxConfig::MAX_NEIGHBORS = RRTxConfigRoot["MAX_NEIGHBORS"].as<size_t>();
    else
        LOG(INFO) << "RRTxConfig::MAX_NEIGHBORS is not defined! Using default value of " << RRTxConfig::MAX_NEIGHBORS;

    if (RRTxConfigRoot["REPLANNING_THROTTLE"].IsDefined())
        RRTxConfig::REPLANNING_THROTTLE = RRTxConfigRoot["REPLANNING_THROTTLE"].as<size_t>();
    else
        LOG(INFO) << "RRTxConfig::REPLANNING_THROTTLE is not defined! Using default value of " << RRTxConfig::REPLANNING_THROTTLE;

    if (RRTxConfigRoot["START_BIAS"].IsDefined())
        RRTxConfig::START_BIAS = RRTxConfigRoot["START_BIAS"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::START_BIAS is not defined! Using default value of " << RRTxConfig::START_BIAS;

    if (RRTxConfigRoot["RESOLUTION_COLL_CHECK"].IsDefined())
        RRTxConfig::RESOLUTION_COLL_CHECK = RRTxConfigRoot["RESOLUTION_COLL_CHECK"].as<float>();
    else
        LOG(INFO) << "RRTxConfig::RESOLUTION_COLL_CHECK is not defined! Using default value of " << RRTxConfig::RESOLUTION_COLL_CHECK;

    if (RRTxConfigRoot["TRAJECTORY_INTERPOLATION"].IsDefined())
        RRTxConfig::TRAJECTORY_INTERPOLATION = planning::trajectory_interpolation_map[RRTxConfigRoot["TRAJECTORY_INTERPOLATION"].as<std::string>()];
    else
        LOG(INFO) << "RRTxConfig::TRAJECTORY_INTERPOLATION is not defined! Using default value of " << RRTxConfig::TRAJECTORY_INTERPOLATION;
    
    // TrajectoryConfigRoot
    if (TrajectoryConfigRoot["MAX_TIME_COMPUTE_REGULAR"].IsDefined())
        TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR = TrajectoryConfigRoot["MAX_TIME_COMPUTE_REGULAR"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR is not defined! Using default value of " << TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR;

    if (TrajectoryConfigRoot["MAX_TIME_COMPUTE_SAFE"].IsDefined())
        TrajectoryConfig::MAX_TIME_COMPUTE_SAFE = TrajectoryConfigRoot["MAX_TIME_COMPUTE_SAFE"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::MAX_TIME_COMPUTE_SAFE is not defined! Using default value of " << TrajectoryConfig::MAX_TIME_COMPUTE_SAFE;
    
    if (TrajectoryConfigRoot["MAX_TIME_FINAL"].IsDefined())
        TrajectoryConfig::MAX_TIME_FINAL = TrajectoryConfigRoot["MAX_TIME_FINAL"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::MAX_TIME_FINAL is not defined! Using default value of " << TrajectoryConfig::MAX_TIME_FINAL;

    if (TrajectoryConfigRoot["TIME_STEP"].IsDefined())
        TrajectoryConfig::TIME_STEP = TrajectoryConfigRoot["TIME_STEP"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::TIME_STEP is not defined! Using default value of " << TrajectoryConfig::TIME_STEP;

    if (TrajectoryConfigRoot["FINAL_JERK_STEP"].IsDefined())
        TrajectoryConfig::FINAL_JERK_STEP = TrajectoryConfigRoot["FINAL_JERK_STEP"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::FINAL_JERK_STEP is not defined! Using default value of " << TrajectoryConfig::FINAL_JERK_STEP;
    
    if (TrajectoryConfigRoot["FINAL_VELOCITY_STEP"].IsDefined())
        TrajectoryConfig::FINAL_VELOCITY_STEP = TrajectoryConfigRoot["FINAL_VELOCITY_STEP"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::FINAL_VELOCITY_STEP is not defined! Using default value of " << TrajectoryConfig::FINAL_VELOCITY_STEP;
    
    if (TrajectoryConfigRoot["MAX_RADIUS"].IsDefined())
        TrajectoryConfig::MAX_RADIUS = TrajectoryConfigRoot["MAX_RADIUS"].as<float>();
    else
        LOG(INFO) << "TrajectoryConfig::MAX_RADIUS is not defined! Using default value of " << TrajectoryConfig::MAX_RADIUS;
    
    if (TrajectoryConfigRoot["SCALE_TARGET"].IsDefined())
        TrajectoryConfig::SCALE_TARGET = TrajectoryConfigRoot["SCALE_TARGET"].as<bool>();
    else
        LOG(INFO) << "TrajectoryConfig::SCALE_TARGET is not defined! Using default value of " << TrajectoryConfig::SCALE_TARGET;
    
    // RT_RGBTConfigRoot
    if (RT_RGBTConfigRoot["MAX_NUM_ITER"].IsDefined())
        RT_RGBTConfig::MAX_NUM_ITER = RT_RGBTConfigRoot["MAX_NUM_ITER"].as<size_t>();
    else
        LOG(INFO) << "RT_RGBTConfig::MAX_NUM_ITER is not defined! Using default value of " << RT_RGBTConfig::MAX_NUM_ITER;
    
    if (RT_RGBTConfigRoot["MAX_ITER_TIME"].IsDefined())
        RT_RGBTConfig::MAX_ITER_TIME = RT_RGBTConfigRoot["MAX_ITER_TIME"].as<float>();
    else
        LOG(INFO) << "RT_RGBTConfig::MAX_ITER_TIME is not defined! Using default value of " << RT_RGBTConfig::MAX_ITER_TIME;
    
    if (RT_RGBTConfigRoot["MAX_PLANNING_TIME"].IsDefined())
        RT_RGBTConfig::MAX_PLANNING_TIME = RT_RGBTConfigRoot["MAX_PLANNING_TIME"].as<float>();
    else
        LOG(INFO) << "RT_RGBTConfig::MAX_PLANNING_TIME is not defined! Using default value of " << RT_RGBTConfig::MAX_PLANNING_TIME;

    if (RT_RGBTConfigRoot["RESOLUTION_COLL_CHECK"].IsDefined())
        RT_RGBTConfig::RESOLUTION_COLL_CHECK = RT_RGBTConfigRoot["RESOLUTION_COLL_CHECK"].as<float>();
    else
        LOG(INFO) << "RT_RGBTConfig::RESOLUTION_COLL_CHECK is not defined! Using default value of " << RT_RGBTConfig::RESOLUTION_COLL_CHECK;
    
    if (RT_RGBTConfigRoot["GOAL_PROBABILITY"].IsDefined())
        RT_RGBTConfig::GOAL_PROBABILITY = RT_RGBTConfigRoot["GOAL_PROBABILITY"].as<float>();
    else
        LOG(INFO) << "RT_RGBTConfig::GOAL_PROBABILITY is not defined! Using default value of " << RT_RGBTConfig::GOAL_PROBABILITY;
    
    if (RT_RGBTConfigRoot["TRAJECTORY_INTERPOLATION"].IsDefined())
        RT_RGBTConfig::TRAJECTORY_INTERPOLATION = planning::trajectory_interpolation_map[RT_RGBTConfigRoot["TRAJECTORY_INTERPOLATION"].as<std::string>()];
    else
        LOG(INFO) << "RT_RGBTConfig::TRAJECTORY_INTERPOLATION is not defined! Using default value of " << RT_RGBTConfig::TRAJECTORY_INTERPOLATION;
    
    LOG(INFO) << "Configuration parameters read successfully!";
}
