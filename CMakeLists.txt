PROJECT(mrpt_vrep_bridge LANGUAGES C CXX)

# CMake project configuration
################################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 3.3)

# CMake Policies
# Warning: Some of these differ from the original MRPT project
cmake_policy(SET CMP0003 NEW) # We don't want to mix relative and absolute paths in linker lib lists.
cmake_policy(SET CMP0005 NEW) # Escape definitions (-D) strings
cmake_policy(SET CMP0053 NEW) # Simplify variable reference and escape sequence evaluation.
cmake_policy(SET CMP0037 NEW)
cmake_policy(SET CMP0043 NEW)
cmake_policy(SET CMP0048 NEW)

# Package version management
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION_TWEAK 0)
string(CONCAT PROJECT_VERSION
    "${PROJECT_VERSION_MAJOR}."
    "${PROJECT_VERSION_MINOR}."
    "${PROJECT_VERSION_PATCH}."
    "${PROJECT_VERSION_TWEAK}")
message(STATUS "${PROJECT_NAME} Version: ${PROJECT_VERSION}")

# Where to place compilation targets
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(test_output_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test)

# Minimum compiler requirements
# same as MRPT-2.X
set(MIN_GCC_CXX_COMPILER_VERSION "7.0.0")
set(MIN_CLANG_CXX_COMPILER_VERSION "4.0.0")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    if (NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_GREATER
            ${MIN_CLANG_CXX_COMPILER_VERSION})
    endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if (NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_GREATER
            ${MIN_GCC_CXX_COMPILER_VERSION})
    endif()
else()
    message(WARNING
        "Compiler \"${CMAKE_CXX_COMPILER_ID}\" is not yet supported by ${PROJECT_NAME}")
endif()


set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#MRPT Lib
################################################################################
find_package(MRPT REQUIRED graphslam)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")



# User Options
################################################################################

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_TESTING "Build all gtests" ON)

# Set the C/CXX Standards that we follow
set (CMAKE_CXX_STANDARD 14)  # Require C++14

# cmake helper scripts
################################################################################

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # Directory for CMake scripts
include(cmake/UtilFuns.cmake REQUIRED)

# External Dependencies
################################################################################

set(third_party "${CMAKE_SOURCE_DIR}/3rdparty/")
set(EXTERNAL_DEPS "")

if(BUILD_TESTING)
  add_subdirectory("${third_party}/googletest")
  enable_testing()
endif()

# doxygen
# Generate documentation using Doxygen
find_package(Doxygen QUIET)

if (DOXYGEN_FOUND)
  add_subdirectory(doc)
endif (DOXYGEN_FOUND)

# Include rest of project components
################################################################################

# Add the various subdirectories
add_subdirectory(mrpt_graphslam_2d)
add_subdirectory(conversions)


# Configure files
################################################################################

configure_file(cmake/${PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
# TODO - How to use it
configure_file(cmake/${PROJECT_NAME}ConfigVersion.hpp.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hpp" @ONLY)
