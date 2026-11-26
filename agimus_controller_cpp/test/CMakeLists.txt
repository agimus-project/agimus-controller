find_package(doctest REQUIRED)
include(${doctest_DIR}/doctest.cmake) # doctest_discover_tests
find_package(example-robot-data REQUIRED)
find_package(tomlplusplus REQUIRED) # Golden files (test/golden); tests only.
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
    test_cost_updater
    test_fixed_capacity
    test_mpc_data
    test_ocp
    test_ocp_params
    test_robot_models
    test_robot_models_more
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
# Fixtures of test_robot_models(_more): their own translation unit, to keep
# each test file under the compile-RAM limit.
add_library(agimus_robot_models_fixtures STATIC robot_models_fixtures.cpp)
target_link_libraries(
    agimus_robot_models_fixtures
    PUBLIC agimus_controller_cpp doctest::doctest
)
target_compile_definitions(
    agimus_robot_models_fixtures
    PRIVATE AGIMUS_EXAMPLE_ROBOT_DATA_DIR="${AGIMUS_EXAMPLE_ROBOT_DATA_DIR}"
)
target_compile_options(agimus_robot_models_fixtures PRIVATE -Wall -Wextra)
foreach(test_name test_robot_models test_robot_models_more)
    target_link_libraries(${test_name} PRIVATE agimus_robot_models_fixtures)
endforeach()

# Panda fixtures of test_ocp and test_cost_updater: their own translation
# unit, to keep each test file under the compile-RAM limit.
add_library(agimus_panda_ocp_fixtures STATIC panda_ocp.cpp)
target_link_libraries(agimus_panda_ocp_fixtures PUBLIC agimus_controller_cpp)
target_compile_definitions(
    agimus_panda_ocp_fixtures
    PRIVATE AGIMUS_EXAMPLE_ROBOT_DATA_DIR="${AGIMUS_EXAMPLE_ROBOT_DATA_DIR}"
)
target_compile_options(agimus_panda_ocp_fixtures PRIVATE -Wall -Wextra)
foreach(test_name test_cost_updater test_ocp)
    target_link_libraries(${test_name} PRIVATE agimus_panda_ocp_fixtures)
endforeach()

# Golden files written by test/golden/generate_golden.py.
foreach(test_name test_ocp test_ocp_params)
    target_link_libraries(${test_name} PRIVATE tomlplusplus::tomlplusplus)
    target_compile_definitions(
        ${test_name}
        PRIVATE AGIMUS_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
    )
endforeach()

add_test(
    NAME check_rt_coverage
    COMMAND
        Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/check_rt_coverage.py
        --include ${PROJECT_SOURCE_DIR}/include --tests
        ${CMAKE_CURRENT_SOURCE_DIR}
)
