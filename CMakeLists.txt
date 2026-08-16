cmake_minimum_required(VERSION 3.15)
project(inertial_navigation_system VERSION 0.1 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(INS_BUILD_TESTS "Build INS and closed-loop integration tests" ON)

set(STATE_ESTIMATION_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CS_BUILD_TESTS OFF CACHE BOOL "" FORCE)

add_subdirectory(external/stateEstimation ${CMAKE_BINARY_DIR}/stateEstimation)
add_subdirectory(external/controlSystems ${CMAKE_BINARY_DIR}/controlSystems)
add_subdirectory(external/dynamic_models ${CMAKE_BINARY_DIR}/dynamic_models)

add_library(ins STATIC src/ins.c)
target_include_directories(ins PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ins PUBLIC state_estimation m)

if(INS_BUILD_TESTS)
    enable_testing()

    add_executable(test_ins tests/test_ins.c)
    target_link_libraries(test_ins PRIVATE ins)
    add_test(NAME test_ins COMMAND test_ins)

    add_executable(test_closed_loop_attitude tests/test_closed_loop_attitude.c)
    target_link_libraries(test_closed_loop_attitude PRIVATE
        ins control_systems dynamic_models attitude m)
    add_test(NAME test_closed_loop_attitude COMMAND test_closed_loop_attitude)
endif()

install(TARGETS ins ARCHIVE DESTINATION lib)
install(FILES include/ins.h DESTINATION include)
