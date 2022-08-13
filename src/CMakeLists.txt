add_subdirectory(carpet)

if(BUILD_MT_RRT_TESTS)
    add_subdirectory(test-scenarios)
endif()

add_subdirectory(core)
option(BUILD_MT_RRT_MULTI_THREADED "Build MT-RRT multi threaded module" ON)
if(BUILD_MT_RRT_MULTI_THREADED)
    add_subdirectory(multi-threaded)
endif()
