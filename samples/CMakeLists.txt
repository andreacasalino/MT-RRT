if(BUILD_MT_RRT_TESTS OR BUILD_MT_RRT_SAMPLES)
    FECTH_JSON_LIB()
    add_subdirectory(utils)
endif()

add_subdirectory(TrivialProblems)
# add_subdirectory(PlanarRobotsProblems)
# add_subdirectory(NavigationProblems)

# add_executable(MT-RRT-ReadMe ReadMe.cpp)
# target_link_libraries(MT-RRT-ReadMe PUBLIC MT-RRT-multi-threaded) 
