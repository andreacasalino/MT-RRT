function(MakeLibrary LIBRARY_NAME)
	file(GLOB_RECURSE SOURCES 
		${CMAKE_CURRENT_SOURCE_DIR}/header/*.h 
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
	)

	if(LIB_OPT)
		if (WIN32)
			set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
		endif ()	
		
		add_library(${LIBRARY_NAME} SHARED ${SOURCES})
	else()
		add_library(${LIBRARY_NAME} STATIC ${SOURCES})
	endif()

	target_include_directories(${LIBRARY_NAME} PUBLIC 
		${CMAKE_CURRENT_SOURCE_DIR}/header
	)

	install(TARGETS ${LIBRARY_NAME})
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/header/ DESTINATION include/${LIBRARY_NAME} FILES_MATCHING PATTERN "*.h*")
endfunction()



function(MakeTestableLibrary LIBRARY_NAME)
	MakeLibrary(${LIBRARY_NAME})

	if(BUILD_MT_RRT_TESTS)
		MAKE_TEST(${LIBRARY_NAME})
	endif()
endfunction()
