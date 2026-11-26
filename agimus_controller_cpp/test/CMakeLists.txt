find_package(doctest REQUIRED)
include(${doctest_DIR}/doctest.cmake) # doctest_discover_tests
find_package(example-robot-data REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Allocation hook, RT guard and the single doctest main. An OBJECT library, so
# that the replaced malloc/new are always linked into every test executable.
add_library(
    agimus_rt_test_support
    OBJECT
    rt/allocation_hook.cpp
    rt/doctest_main.cpp
    rt/rt_guard.cpp
)
target_include_directories(agimus_rt_test_support PUBLIC rt)
target_link_libraries(
    agimus_rt_test_support
    PUBLIC agimus_controller_cpp doctest::doctest
)

set(AGIMUS_TESTS
    test_fixed_capacity
    test_mpc_data
    test_ocp_params
    test_robot_models
    test_rt_harness
    test_trajectory_buffer
    test_trajectory_point
)
foreach(test_name ${AGIMUS_TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE agimus_rt_test_support)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra)
    doctest_discover_tests(${test_name})
endforeach()

# example-robot-data's config only provides its include directory: the models
# are in <prefix>/share/example-robot-data.
get_filename_component(
    AGIMUS_EXAMPLE_ROBOT_DATA_DIR
    "${example-robot-data_INCLUDE_DIRS}/../share/example-robot-data"
    ABSOLUTE
)
target_compile_definitions(
    test_robot_models
    PRIVATE AGIMUS_EXAMPLE_ROBOT_DATA_DIR="${AGIMUS_EXAMPLE_ROBOT_DATA_DIR}"
)

add_test(
    NAME check_rt_coverage
    COMMAND
        Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/check_rt_coverage.py
        --include ${PROJECT_SOURCE_DIR}/include --tests
        ${CMAKE_CURRENT_SOURCE_DIR}
)
