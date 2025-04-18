add_executable(test_nanoflann test_nanoflann.cpp)
target_compile_features(test_nanoflann PRIVATE cxx_std_17)
target_link_libraries(test_nanoflann PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_kdl_parser test_kdl_parser.cpp)
target_compile_features(test_kdl_parser PRIVATE cxx_std_17)
target_link_libraries(test_kdl_parser PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_planar_2dof test_planar_2dof.cpp)
target_compile_features(test_planar_2dof PRIVATE cxx_std_17)
target_link_libraries(test_planar_2dof PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_planar_2dof PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_xarm6 test_xarm6.cpp)
target_compile_features(test_xarm6 PRIVATE cxx_std_17)
target_link_libraries(test_xarm6 PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_xarm6 PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_obstacle_parsing test_obstacle_parsing.cpp)
target_compile_features(test_obstacle_parsing PRIVATE cxx_std_17)
target_link_libraries(test_obstacle_parsing PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_real_vector_space test_real_vector_space.cpp)
target_compile_features(test_real_vector_space PRIVATE cxx_std_17)
target_link_libraries(test_real_vector_space PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_real_vector_space PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_fcl_distance test_fcl_distance.cpp)
target_compile_features(test_fcl_distance PRIVATE cxx_std_17)
target_link_libraries(test_fcl_distance PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rrtconnect test_rrtconnect.cpp)
target_compile_features(test_rrtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rrtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rrtconnect PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_rbtconnect test_rbtconnect.cpp)
target_compile_features(test_rbtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rbtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rbtconnect PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_rgbtconnect test_rgbtconnect.cpp)
target_compile_features(test_rgbtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rgbtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rgbtconnect PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_drgbt test_drgbt.cpp)
target_compile_features(test_drgbt PRIVATE cxx_std_17)
target_link_libraries(test_drgbt PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_drgbt PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_drgbt_random_obstacles test_drgbt_random_obstacles.cpp)
target_compile_features(test_drgbt_random_obstacles PRIVATE cxx_std_17)
target_link_libraries(test_drgbt_random_obstacles PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_drgbt_random_obstacles PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_rgbmtstar test_rgbmtstar.cpp)
target_compile_features(test_rgbmtstar PRIVATE cxx_std_17)
target_link_libraries(test_rgbmtstar PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rgbmtstar PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(generate_random_scenarios generate_random_scenarios.cpp)
target_compile_features(generate_random_scenarios PRIVATE cxx_std_17)
target_link_libraries(generate_random_scenarios PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(generate_random_scenarios PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(generate_dataset generate_dataset.cpp)
target_compile_features(generate_dataset PRIVATE cxx_std_17)
target_link_libraries(generate_dataset PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(generate_dataset PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_rrtx test_rrtx.cpp)
target_compile_features(test_rrtx PRIVATE cxx_std_17)
target_link_libraries(test_rrtx PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rrtx PUBLIC ${PROJECT_SOURCE_DIR}/apps)

add_executable(test_rrtx_random_obstacles test_rrtx_random_obstacles.cpp)
target_compile_features(test_rrtx_random_obstacles PRIVATE cxx_std_17)
target_link_libraries(test_rrtx_random_obstacles PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(test_rrtx_random_obstacles PUBLIC ${PROJECT_SOURCE_DIR}/apps)

install(TARGETS
  test_nanoflann
  test_kdl_parser
  test_planar_2dof
  test_xarm6
  test_obstacle_parsing
  test_real_vector_space
  test_fcl_distance
  test_rrtconnect
  test_rbtconnect
  test_rgbtconnect
  test_drgbt
  test_drgbt_random_obstacles
  test_rgbmtstar
  generate_random_scenarios
  generate_dataset
  test_rrtx
  test_rrtx_random_obstacles
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)