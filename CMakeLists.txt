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

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") # Flags specific to Clang
  if (NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_GREATER
      ${MIN_CLANG_CXX_COMPILER_VERSION})
    message(FATAL_ERROR
      "Compiler version [${CMAKE_CXX_COMPILER_VERSION}] isn't supported. Minimum: ${MIN_CLANG_CXX_COMPILER_VERSION}")
  endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") # Flags specific to GCC
  if (NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_GREATER
      ${MIN_GCC_CXX_COMPILER_VERSION})
    message(FATAL_ERROR
      "Compiler version [${CMAKE_CXX_COMPILER_VERSION}] isn't supported. Minimum: ${MIN_GCC_CXX_COMPILER_VERSION}")
  endif()
else()
  # TODO - make it run on Windows
  message(WARNING
    "Compiler [${CMAKE_CXX_COMPILER_ID}] is not yet supported by ${PROJECT_NAME}")
endif()


set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the standard
set (CMAKE_C_STANDARD 99)  # Require C99
set (CMAKE_CXX_STANDARD 17)  # Require C++17

#MRPT Lib
################################################################################
find_package(MRPT REQUIRED graphslam)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

# TODO - Link the mrpt libraries to the excecutable created by the VREP files 


# User Options
################################################################################

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_TESTING "Build all gtests" ON)

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

#V-REP Config
################################################################################

if (EXISTS "$ENV{VREP}")
  message(STATUS "Found VREP")
  set (VREP_ROOT "$ENV{VREP}" CACHE "VREP root dir" STRING)
  set (VREP_PATH "$ENV{VREP}/programming" CACHE "The VREP build path" STRING)
  set (VREP_SOURCES
    ${VREP_PATH}/remoteApi/extApi.h ${VREP_PATH}/remoteApi/extApiPlatform.h ${VREP_PATH}/include/shared_memory.h
    ${VREP_PATH}/remoteApi/extApi.c ${VREP_PATH}/common/shared_memory.c ${VREP_PATH}/remoteApi/extApiPlatform.c
  )
  include_directories (${VREP_PATH}/remoteApi ${VREP_PATH}/include)
  add_definitions( -Dpthread )
  add_definitions (-DNON_MATLAB_PARSING)
  add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
  add_library (vrep_lib SHARED ${VREP_SOURCES})
else()
  message(FATAL_ERROR "Can't detect V-REP installation. Have you exported the VREP environment variable to the corresponding path?")
endif()