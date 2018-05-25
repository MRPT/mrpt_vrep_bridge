PROJECT(mrpt_graphslam_2d)
set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)


if (BUILD_TESTING)
  set(test_sources
  	include/mrpt_bridge/vrep_conversion.h
  	remoteApi/vrep_conversion.cpp  
  	test/runTests.cpp
  	test/convertTest.cpp
  	)
  add_executable(${PROJECT_TEST_NAME} ${test_sources})
  target_link_libraries(${PROJECT_TEST_NAME}
    gtest
    pthread
    ${MRPT_LIBS}
    )
  set_target_properties(
    ${PROJECT_TEST_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${test_output_directory}
    )
  add_test(NAME ${PROJECT_TEST_NAME} COMMAND
    ${test_output_directory}/${PROJECT_TEST_NAME})
endif()


#V-REP Config
################################################################################

if (EXISTS "$ENV{VREP}")
  message(STATUS "Found VREP")
  set (VREP_ROOT "$ENV{VREP}" CACHE "VREP root dir" STRING)


  set (VREP_PATH "$ENV{VREP}/programming" CACHE "The VREP build path" STRING)

  set (SOURCES
  	include/mrpt_bridge/vrep_conversion.h 
    ${VREP_PATH}/remoteApi/extApi.h
    ${VREP_PATH}/remoteApi/extApiPlatform.h
    ${VREP_PATH}/include/shared_memory.h
    ${VREP_PATH}/remoteApi/extApi.c
    ${VREP_PATH}/common/shared_memory.c
    ${VREP_PATH}/remoteApi/extApiPlatform.c
    remoteApi/vrep_conversion.cpp)

  include_directories (${VREP_PATH}/remoteApi ${VREP_PATH}/include include)
  
  add_definitions( -Dpthread )
  add_definitions (-DNON_MATLAB_PARSING)
  add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
  
  add_library (test_Client SHARED ${SOURCES})
  
  
  add_executable(remoteApi_test remoteApi/remoteApi_test.cpp)
  
  target_link_libraries (
  	remoteApi_test 
  	test_Client 
  	pthread
  	rt
    ${MRPT_LIBS}
    )

  
else()
  message(FATAL_ERROR "Can't detect V-REP installation. Have you exported the VREP environment variable to the corresponding path?")

endif()  
