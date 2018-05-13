PROJECT(mrpt_graphslam_2d)
set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)


if (BUILD_TESTING)
  set(test_sources "test/runTests.cpp")
  add_executable(${PROJECT_TEST_NAME}
    ${test_sources})
  target_link_libraries(${PROJECT_TEST_NAME}
    gtest
    pthread)
  set_target_properties(
    ${PROJECT_TEST_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${test_output_directory}
    )
  add_test(NAME ${PROJECT_TEST_NAME} COMMAND
    ${test_output_directory}/${PROJECT_TEST_NAME})
endif()

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

#V-REP Config
################################################################################

if (EXISTS "${PROJECT_ROOT_DIR}/../V-REP")
  message(STATUS "Found VREP")
  set (V-REP_ROOT "${PROJECT_ROOT_DIR}/../" CACHE "V-REP root dir" STRING)

  execute_process (COMMAND "/bin/bash" "-c" "cd ${V-REP_ROOT}; echo -n `pwd`" OUTPUT_VARIABLE V-REP_ROOT)

  #Create symbolic links from Addon to V-REP directory
  execute_process (COMMAND "/bin/bash" "-c" "cd ${V-REP_ROOT}; Addon/links.sh ${V-REP_ROOT}")

  set (V-REP_PATH "${PROJECT_ROOT_DIR}/../V-REP/programming/" CACHE "The V-REP build path" STRING)
  set (SOURCES remoteApi/bubbleRob_test.cpp
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApi.c
    ${V-REP_ROOT}/V-REP/programming/remoteApi/shared_memory.c
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApiPlatform.c)

  set (EXTRA_LIBRARIES pthread)
  include_directories (${V-REP_PATH}/remoteApi ${V-REP_PATH}/include)
  add_definitions (-DNON_MATLAB_PARSING=1)
  add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
  add_library (V-REPClient SHARED ${SOURCES})
  target_link_libraries (V-REPClient ${EXTRA_LIBRARIES})

  #Compile the api
  #execute_process (COMMAND "/bin/bash" "-c" "cd ${V-REP_ROOT}/V-REP/programming/v-repSampleApi/mrpt_graphslam_2d/remoteApi; qmake bubbleRob_test.pro; make")

else()
  message(STATUS "Kindly Install VREP")

endif(EXISTS "${PROJECT_ROOT_DIR}/../V-REP")  
