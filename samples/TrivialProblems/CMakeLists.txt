if(BUILD_MT_RRT_TESTS OR BUILD_MT_RRT_SAMPLES)
   add_subdirectory(src)
endif()

if(BUILD_MT_RRT_SAMPLES)
   add_subdirectory(scenarios)
endif()
