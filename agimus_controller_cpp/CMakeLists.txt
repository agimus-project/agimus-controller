cmake_minimum_required(VERSION 3.22)
project(agimus_controller_cpp VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(CTest) # Defines BUILD_TESTING (ON by default).
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(
    AGIMUS_CONTROLLER_RT_CHECKS
    "Real-time checks: EIGEN_RUNTIME_NO_MALLOC, NaN-initialized Eigen objects and assertions on. Not for production or benchmarks."
    ${BUILD_TESTING}
)

if(POLICY CMP0167)
    # pinocchio and coal look Boost up through its own CMake config.
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Minimums: the oldest version on any supported platform (ledger, T0.5).
find_package(pinocchio 4.1.0 REQUIRED)
find_package(coal 3.0.3 REQUIRED)
# crocoddyl's config links OpenMP::OpenMP_CXX without looking it up.
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(crocoddyl 3.2.0 REQUIRED)
find_package(mim_solvers 0.3.0 REQUIRED)

add_library(
    agimus_controller_cpp
    src/cost_updater.cpp
    src/mpc_data.cpp
    src/ocp.cpp
    src/ocp_builders.cpp
    src/ocp_params.cpp
    src/robot_models.cpp
    src/status.cpp
    src/trajectory_buffer.cpp
    src/trajectory_point.cpp
    src/warm_start.cpp
)
add_library(
    agimus_controller_cpp::agimus_controller_cpp
    ALIAS agimus_controller_cpp
)
target_include_directories(
    agimus_controller_cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    agimus_controller_cpp
    PUBLIC
        Eigen3::Eigen
        coal::coal
        crocoddyl::crocoddyl
        mim_solvers::mim_solvers
        pinocchio::pinocchio
)
target_compile_options(agimus_controller_cpp PRIVATE -Wall -Wextra -Wpedantic)
if(AGIMUS_CONTROLLER_RT_CHECKS)
    # PUBLIC so that tests and library agree on the Eigen configuration, but
    # build-tree only: installed consumers never inherit the checks.
    # EIGEN_NO_AUTOMATIC_RESIZING is not used: Eigen 3.4 ignores it on plain
    # assignment; a resize under the RT guard is caught as an allocation.
    # EIGEN_INITIALIZE_MATRICES_BY_NAN turns reads of never-written values
    # into NaN, so they fail the tests.
    target_compile_definitions(
        agimus_controller_cpp
        PUBLIC
            $<BUILD_INTERFACE:EIGEN_RUNTIME_NO_MALLOC>
            $<BUILD_INTERFACE:EIGEN_INITIALIZE_MATRICES_BY_NAN>
    )
    # Eigen's checks are eigen_assert: keep assertions on.
    target_compile_options(
        agimus_controller_cpp
        PUBLIC $<BUILD_INTERFACE:-UNDEBUG>
    )
endif()

install(
    TARGETS agimus_controller_cpp
    EXPORT agimus_controller_cppTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
    EXPORT agimus_controller_cppTargets
    NAMESPACE agimus_controller_cpp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/agimus_controller_cpp
)
configure_package_config_file(
    cmake/agimus_controller_cppConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/agimus_controller_cppConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/agimus_controller_cpp
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/agimus_controller_cppConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/agimus_controller_cppConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/agimus_controller_cppConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/agimus_controller_cpp
)
install(FILES package.xml DESTINATION share/agimus_controller_cpp)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
