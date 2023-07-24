# Testing library


include_directories(include)

# Tests need to be added as executables first
add_executable(main_test main_test.cpp)

# I'm using C++17 in the test
target_compile_features(main_test PRIVATE cxx_std_17)

# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(main_test PRIVATE rpmpl_library ${PROJECT_LIBRARIES})

# If you register a test, then ctest and make test will run it.
# You can also run examples and check the output, as well.
add_test(NAME main_test COMMAND main_test) # Command can be a target
