add_subdirectory(Utils)

option(BUILD_SAMPLE_01 "Build point2d samples" ON)
if(BUILD_SAMPLE_01)
   add_subdirectory(01_Point2D)
endif()
