# Minimum CMake version required
cmake_minimum_required(VERSION 3.20)

# Project Name and Version
project(InertialNavigationSystem VERSION 1.0 LANGUAGES C)

# Enable Testing
enable_testing()

# Set C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Option to build for embedded systems (single binary)
option(BUILD_FOR_EMBEDDED "Build as a single binary for embedded systems" ON)

# Include directories for the project
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/stateEstimation/include
    ${CMAKE_SOURCE_DIR}/external/controlSystems/include
    ${CMAKE_SOURCE_DIR}/external/dynamic_models/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/stateEstimation/external/attitudeMathLibrary/include  
)

# Gather source files
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.c
    ${CMAKE_SOURCE_DIR}/src/wrappers/*.c
)

# Add submodule source files directly if building for embedded systems
if(BUILD_FOR_EMBEDDED)
    message(STATUS "Building as a single binary for embedded systems")
    file(GLOB_RECURSE STATE_EST_SRC ${CMAKE_SOURCE_DIR}/external/stateEstimation/src/*.c)
    file(GLOB_RECURSE CONTROL_SYS_SRC ${CMAKE_SOURCE_DIR}/external/controlSystems/src/*.c)
    file(GLOB_RECURSE DYNAMIC_MODELS_SRC ${CMAKE_SOURCE_DIR}/external/dynamic_models/src/*.c)
    list(APPEND SRC_FILES ${STATE_EST_SRC} ${CONTROL_SYS_SRC} ${DYNAMIC_MODELS_SRC})
else()
    # Add external libraries as subdirectories
    add_subdirectory(external/stateEstimation)
    add_subdirectory(external/controlSystems)
    add_subdirectory(external/dynamic_models)
endif()

# Create the INS Library
add_library(ins_library STATIC ${SRC_FILES})

# Link external libraries if not building for embedded systems
if(NOT BUILD_FOR_EMBEDDED)
    target_link_libraries(ins_library PUBLIC stateEstimation controlSystems dynamic_models)
endif()

# Tests
# Add test executables and link the INS Library
file(GLOB TEST_FILES ${CMAKE_SOURCE_DIR}/tests/*.c)
foreach(test_file ${TEST_FILES})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name} PRIVATE ins_library)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# Examples
# Add example executables
file(GLOB EXAMPLE_FILES ${CMAKE_SOURCE_DIR}/examples/*.c)
foreach(example_file ${EXAMPLE_FILES})
    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file})
    target_link_libraries(${example_name} PRIVATE ins_library)
endforeach()

# Installation
install(TARGETS ins_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Packaging (optional)
include(CPack)
