add_subdirectory(Utils)

option(BUILD_SAMPLE_01 "Build point-like states samples" ON)
if(BUILD_SAMPLE_01)
   add_subdirectory(01-Points)
endif()

option(BUILD_SAMPLE_02 "Build planar manipulators samples" ON)
if(BUILD_SAMPLE_02)
   add_subdirectory(02-Manipulators)
endif()
