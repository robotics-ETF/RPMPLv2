cmake_minimum_required(VERSION 3.12)

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

# Project name and a few useful settings. Other commands can pick up the results
project(
  RPMPL
  VERSION 0.1
  DESCRIPTION "Rapid Prototyping Motion Planning Library"
  LANGUAGES CXX)

# set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("------------------ Using compiler: ${CMAKE_CXX_COMPILER} ------------------")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Let's nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  # Testing only available if this is the main app
  # Note this needs to be done in the main CMakeLists
  # since it calls enable_testing, which must be in the
  # main CMakeLists.
  include(CTest)

  # Docs only available if this is the main app
  find_package(Doxygen)

  if(Doxygen_FOUND)
    add_subdirectory(docs)
  else()
    message(STATUS "Doxygen not found, not building docs")
  endif()
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
include_directories(${OROCOS_KDL_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)
find_package(fcl 0.7 REQUIRED)
find_package(nanoflann REQUIRED)

set(PROJECT_LIBRARIES gtest glog gflags nanoflann::nanoflann kdl_parser orocos-kdl fcl ccd yaml-cpp)

set(MAIN_PROJECT_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/apps/data)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/visualizer/
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/apps/visualizer)

# The compiled library code is here
add_subdirectory(src)

# The executable code is here
add_subdirectory(apps)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING) AND BUILD_TESTING)
  add_subdirectory(tests)
endif()