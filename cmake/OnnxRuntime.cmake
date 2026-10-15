# Include this before add_subdirectory(src) and add_subdirectory(apps).
set(ONNXRUNTIME_ROOT "${PROJECT_SOURCE_DIR}/external/onnxruntime"
    CACHE PATH "Path to the extracted ONNX Runtime CPU package")
get_filename_component(ONNXRUNTIME_ROOT "${ONNXRUNTIME_ROOT}" ABSOLUTE)

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h"
   OR NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
    message(FATAL_ERROR
        "ONNX Runtime headers/library not found in ${ONNXRUNTIME_ROOT}. "
        "Extract the Linux CPU package or set -DONNXRUNTIME_ROOT=/absolute/path.")
endif()

if(NOT TARGET ONNXRuntime::ONNXRuntime)
    add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED GLOBAL)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
        IMPORTED_LOCATION "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_ROOT}/include")
endif()

# Find the shared library when running executables from the build directory.
# This is a development build path, not a portable installation configuration.
list(APPEND CMAKE_BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib")