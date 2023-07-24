add_executable(test_nanoflann test_nanoflann.cpp)
target_compile_features(test_nanoflann PRIVATE cxx_std_17)
target_link_libraries(test_nanoflann PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_kdl_parser test_kdl_parser.cpp)
target_compile_features(test_kdl_parser PRIVATE cxx_std_17)
target_link_libraries(test_kdl_parser PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_planar_2dof test_planar_2dof.cpp)
target_compile_features(test_planar_2dof PRIVATE cxx_std_17)
target_link_libraries(test_planar_2dof PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_xarm6 test_xarm6.cpp)
target_compile_features(test_xarm6 PRIVATE cxx_std_17)
target_link_libraries(test_xarm6 PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_obstacle_parsing test_obstacle_parsing.cpp)
target_compile_features(test_obstacle_parsing PRIVATE cxx_std_17)
target_link_libraries(test_obstacle_parsing PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_real_vector_space test_real_vector_space.cpp)
target_compile_features(test_real_vector_space PRIVATE cxx_std_17)
target_link_libraries(test_real_vector_space PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_fcl_distance test_fcl_distance.cpp)
target_compile_features(test_fcl_distance PRIVATE cxx_std_17)
target_link_libraries(test_fcl_distance PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rrtconnect test_rrtconnect.cpp)
target_compile_features(test_rrtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rrtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rrtconnect_benchmark test_rrtconnect_benchmark.cpp)
target_compile_features(test_rrtconnect_benchmark PRIVATE cxx_std_17)
target_link_libraries(test_rrtconnect_benchmark PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rbtconnect test_rbtconnect.cpp)
target_compile_features(test_rbtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rbtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rgbtconnect test_rgbtconnect.cpp)
target_compile_features(test_rgbtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rgbtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_drgbtconnect_benchmark test_drgbtconnect_benchmark.cpp)
target_compile_features(test_drgbtconnect_benchmark PRIVATE cxx_std_17)
target_link_libraries(test_drgbtconnect_benchmark PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rgbmtstar test_rgbmtstar.cpp)
target_compile_features(test_rgbmtstar PRIVATE cxx_std_17)
target_link_libraries(test_rgbmtstar PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rgbmtstar_benchmark test_rgbmtstar_benchmark.cpp)
target_compile_features(test_rgbmtstar_benchmark PRIVATE cxx_std_17)
target_link_libraries(test_rgbmtstar_benchmark PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

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
  test_drgbtconnect_benchmark
  test_rgbmtstar
  test_rgbmtstar_benchmark
  DESTINATION apps)