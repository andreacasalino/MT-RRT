add_subdirectory(Utils)

option(BUILD_SAMPLE_01 "Build point-like states samples" ON)
if(BUILD_SAMPLE_01)
   add_subdirectory(01-Points)
endif()
