PROJECT(${PROJECT_ROOT}_conversions)
set(CONVERT_TEST_NAME mrpt_vrep_bridge_conversions_test)

#Testing
######################################################

if (BUILD_TESTING)
	set(convertTest_sources
		include/mrpt_bridge/vrep_conversion.h
		vrep_conversion.cpp
		test/runTests.cpp
		test/convertTest.cpp
	)
	add_executable(${CONVERT_TEST_NAME} ${convertTest_sources})
	target_link_libraries(${CONVERT_TEST_NAME}
		gtest
    	pthread
    	${MRPT_LIBS}
    )
    set_target_properties(${CONVERT_TEST_NAME}
    	PROPERTIES
    	RUNTIME_OUTPUT_DIRECTORY ${test_output_directory}
    )
    add_test(NAME ${CONVERT_TEST_NAME} COMMAND
    	${test_output_directory}/${CONVERT_TEST_NAME})
endif()

set (SOURCES
	include/mrpt_bridge/vrep_conversion.h
	vrep_conversion.cpp
)
include_directories (include)
add_library (conversion_lib SHARED ${SOURCES})
target_link_libraries(
	conversion_lib
	${MRPT_LIBS}
)