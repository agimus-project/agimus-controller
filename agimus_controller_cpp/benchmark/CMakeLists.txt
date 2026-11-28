# Benchmark of Mpc::run (design.md, "Benchmark"). Not built by default and not
# by the nix package: configure with -DAGIMUS_CONTROLLER_BUILD_BENCHMARKS=ON
# -DCMAKE_BUILD_TYPE=Release -DAGIMUS_CONTROLLER_RT_CHECKS=OFF. It reuses the
# fixtures of the tests (panda, golden set G10a), so BUILD_TESTING must be ON.
if(NOT BUILD_TESTING)
    message(
        FATAL_ERROR
        "AGIMUS_CONTROLLER_BUILD_BENCHMARKS needs BUILD_TESTING (it reuses the test fixtures)."
    )
endif()
if(AGIMUS_CONTROLLER_RT_CHECKS OR NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    message(
        WARNING
        "bench_mpc: build with CMAKE_BUILD_TYPE=Release and AGIMUS_CONTROLLER_RT_CHECKS=OFF for meaningful numbers."
    )
endif()

execute_process(
    COMMAND git -C ${PROJECT_SOURCE_DIR} describe --always --dirty
    OUTPUT_VARIABLE AGIMUS_BENCH_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT AGIMUS_BENCH_COMMIT)
    set(AGIMUS_BENCH_COMMIT unknown)
endif()

add_executable(bench_mpc bench_mpc.cpp)
target_link_libraries(bench_mpc PRIVATE agimus_mpc_fixtures)
target_include_directories(bench_mpc PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(bench_mpc PRIVATE -Wall -Wextra)
target_compile_definitions(
    bench_mpc
    PRIVATE
        AGIMUS_BENCH_COMMIT="${AGIMUS_BENCH_COMMIT}"
        AGIMUS_BENCH_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
        AGIMUS_BENCH_COMPILER="${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
)
