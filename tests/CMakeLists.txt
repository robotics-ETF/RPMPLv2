# Testing library

include_directories(include)

# Tests need to be added as executables first
add_executable(test_realvectorspacestate test_realvectorspacestate.cpp)
add_executable(test_vectorf test_vectorf.cpp)

target_compile_features(test_realvectorspacestate PRIVATE)
target_compile_features(test_vectorf PRIVATE)

# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(test_realvectorspacestate PRIVATE rpmpl_library ${PROJECT_LIBRARIES})
target_link_libraries(test_vectorf PRIVATE rpmpl_library ${PROJECT_LIBRARIES})

# If you register a test, then ctest and make test will run it.
# You can also run examples and check the output, as well.
add_test(NAME test_realvectorspacestate COMMAND test_realvectorspacestate) # Command can be a target
add_test(NAME test_vectorf COMMAND test_vectorf) # Command can be a target
