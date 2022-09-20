add_subdirectory(carpet)

add_subdirectory(core)
option(BUILD_MT_RRT_MULTI_THREADED "Build MT-RRT multi threaded module" ON)
if(BUILD_MT_RRT_MULTI_THREADED)
    add_subdirectory(multi-threaded)
endif()
