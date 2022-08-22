if(BUILD_MT_RRT_TESTS OR BUILD_MT_RRT_SAMPLES)
   add_subdirectory(src)

   if(BUILD_MT_RRT_TESTS)
      add_subdirectory(test-scenarios)
   endif()
endif()

if(BUILD_MT_RRT_SAMPLES)
   add_subdirectory(scenarios)
endif()
