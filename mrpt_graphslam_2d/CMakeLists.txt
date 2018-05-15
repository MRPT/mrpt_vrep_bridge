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

if (EXISTS "$ENV{VREP}")
  message(STATUS "Found V-REP")
  set (V-REP_ROOT "${PROJECT_ROOT_DIR}/../" CACHE "V-REP root dir" STRING)


  set (V-REP_PATH "${PROJECT_ROOT_DIR}/../V-REP/programming/" CACHE "The V-REP build path" STRING)

  set (SOURCES remoteApi/bubbleRob_test.cpp
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApi.c
    ${V-REP_ROOT}/V-REP/programming/common/shared_memory.c
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApi.h
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApiPlatform.h
    ${V-REP_ROOT}/V-REP/programming/include/shared_memory.h
    ${V-REP_ROOT}/V-REP/programming/remoteApi/extApiPlatform.c)

  set (EXTRA_LIBRARIES pthread)

  include_directories (${V-REP_PATH}/remoteApi ${V-REP_PATH}/include)
  
  add_definitions( -Dpthread )
  add_definitions (-DNON_MATLAB_PARSING)
  add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
  
  add_library (test_Client SHARED ${SOURCES})
  
  
  add_executable(bubbleRob_test ${SOURCES} )

  target_link_libraries (bubbleRob_test test_Client ${EXTRA_LIBRARIES})
  target_link_libraries (bubbleRob_test test_Client rt)

  
  #Compile the api
  #execute_process (COMMAND "/bin/bash" "-c" "cd ${V-REP_ROOT}/V-REP/programming/v-repSampleApi; qmake bubbleRob_test.pro; make")

else()
  message(STATUS "Kindly Install V-REP")

endif()  
