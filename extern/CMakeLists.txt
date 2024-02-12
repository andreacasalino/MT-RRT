include(FetchContent)

if(BUILD_MT_RRT_TESTS)
	FetchContent_Declare(
	  googletest
	  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
	)
	if (WIN32)
		# For Windows: Prevent overriding the parent project's compiler/linker settings
		set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	endif (WIN32)
	FetchContent_MakeAvailable(googletest)
endif()

if(BUILD_MT_RRT_TESTS OR BUILD_MT_RRT_SAMPLES)
    Fetch(https://github.com/nlohmann/json.git v3.10.5 json_lib)
endif()